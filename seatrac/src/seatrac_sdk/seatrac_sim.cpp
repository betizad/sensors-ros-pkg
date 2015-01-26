/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, LABUST, UNIZG-FER
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the LABUST nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author : Dula Nad
 *  Created: 23.01.2013.
 *********************************************************************/
#include <labust/seatrac/seatrac_sim.h>
#include <labust/seatrac/seatrac_factory.h>
#include <labust/seatrac/seatrac_definitions.h>
#include <pluginlib/class_list_macros.h>

#include <underwater_msgs/AcSimRegister.h>

using namespace labust::seatrac;

SeatracSim::SeatracSim():
	state(IDLE),
	expected_id(0),
	node_id(1),
	ping_duration(0.7),
	max_distance(500.0),
	vos(1500),
	time_overhead(0.1),
	registered(false){}

SeatracSim::~SeatracSim()
{
	sleeper.stop();
}

bool SeatracSim::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	ph.param("sim_node_id", node_id, node_id);
	ph.param("sim_ping_duration", ping_duration, ping_duration);
	ph.param("sim_max_distance", max_distance, max_distance);
	ph.param("sim_vos", vos, vos);
	ph.param("sim_time_overhead", time_overhead, time_overhead);

	navsts = nh.subscribe<auv_msgs::NavSts>("navsts", 1,
			&SeatracSim::onNavSts, this);
	medium_in = nh.subscribe<underwater_msgs::MediumTransmission>("medium_in", 1,
			&SeatracSim::onMediumTransmission, this);

	medium_out = nh.advertise<underwater_msgs::MediumTransmission>("medium_out",1);

	//Create timer
	sleeper = nh.createTimer(ros::Duration(ping_duration),
			&SeatracSim::onUSBLTimeout, this,
			true, false);

	//Register to medium with node id, navsts topic name, etc.
	registerModem();

	return true;
}

void SeatracSim::registerModem()
{
	if (registered) return;

	ros::NodeHandle nh;
	ros::ServiceClient reg = nh.serviceClient<underwater_msgs::AcSimRegister>("register_modem");

	underwater_msgs::AcSimRegister srv;
	srv.request.node_id = node_id;
	srv.request.navsts_topic = navsts.getTopic();
	registered = reg.call(srv);

	if (!registered)
	{
		ROS_WARN("SeatracSim: modem is not registered with the acoustic medium.");
	}
}

void SeatracSim::onNavSts(const auv_msgs::NavSts::ConstPtr& msg)
{
	boost::shared_ptr<StatusResp> resp(new StatusResp());
	//\todo switch to USBL frame to incorporate offsets
	//Setup flags
	resp->status.status_output.ACC_CAL = 0;
	resp->status.status_output.AHRS_COMP_DATA = 0;
	resp->status.status_output.AHRS_RAW_DATA = 0;
	resp->status.status_output.ATTITUDE = 1;
	resp->status.status_output.ENVIRONMENT = 0;
	resp->status.status_output.MAG_CAL = 0;

	resp->status.attitude[Status::ROLL] = msg->orientation.roll*Status::ATT_SC;
	resp->status.attitude[Status::PITCH] = msg->orientation.pitch*Status::ATT_SC;
	resp->status.attitude[Status::YAW] = msg->orientation.yaw*Status::ATT_SC;

	//Set the current node state
	boost::mutex::scoped_lock ls(position_mux);
	navstate = *msg;
	ls.unlock();

	this->sendMessage(boost::dynamic_pointer_cast<SeatracMessage const>(resp));
}

bool SeatracSim::send(const SeatracMessage::ConstPtr& msg)
{
	//Try registering before sending
	this->registerModem();
	if (!registered) return false;

	if (getState() != IDLE)
	{
		//Reply with device busy
		sendError<PingSendResp>(CST::XCVR::BUSY);
		return true;
	}

	underwater_msgs::MediumTransmission::Ptr tomedium(new underwater_msgs::MediumTransmission());
	tomedium->header.stamp = ros::Time::now();
	tomedium->sender = this->node_id;
	//Broadcast by default
	tomedium->receiver = 0;
	//Minimum duration is ping
	tomedium->duration = ping_duration;

	if (msg->getCid() == PingSendCmd::CID)
	{
		PingSendCmd::ConstPtr cmd = boost::dynamic_pointer_cast<PingSendCmd const>(msg);
		tomedium->receiver = cmd->dest;
		//Start internal wait for ping reply thread
		expected_id = cmd->dest;
		this->setState(WAIT_PING_REPLY);
	}
	else
	{
		ROS_ERROR("SeatracSim: Tried to send unknown message CID.");
		return false;
	}

	//Same for all valid messages
	//Pack ping and send
	//\todo avoid copy ?
	SeatracMessage::DataBuffer buf;
	msg->pack(buf);
	tomedium->message.assign(buf.begin(), buf.end());
	this->sendToMedium(tomedium);
	this->startTimer(2*tomedium->duration + 2*max_distance/vos + time_overhead);

	return true;
}

bool SeatracSim::resend()
{
	ROS_ERROR("SeatracSim: Resend not implemented.");
	return true;
}

void SeatracSim::onMediumTransmission(const
				underwater_msgs::MediumTransmission::ConstPtr& msg)
{
	//Ignore self-messages - there should be no messages
	if (msg->sender == node_id) return;
	//Test if it is possible to hear the message

	//Throw dice if we will have a CRC error

	//Dispatch message based on CID (PING, DATA)
	//If more message types should be handled switch to map dispatch for readability.
	if (msg->message[0] == PingSendCmd::CID)
	{
		processPingCmd(msg);
	}
	else if (msg->message[0] == PingSendCmd::CID)
	{
		//Process data command
	}
	else
	{
		ROS_WARN("Unable to process message CID=0x%x", msg->message[0]);
	}

}

void SeatracSim::processPingCmd(const underwater_msgs::MediumTransmission::ConstPtr& msg)
{
	SeatracMessage::ConstPtr out;
	int state = getState();

	if (state == IDLE)
	{
		//Reply on ping command
		underwater_msgs::MediumTransmission::Ptr rep(new underwater_msgs::MediumTransmission());
		rep->sender = node_id;
		rep->receiver = msg->sender;
		rep->duration = ping_duration;
		rep->message.push_back(PingSendCmd::CID);

		this->sendToMedium(rep);
		//Create PING_REQ message and send to callback
		PingReq::Ptr req(new PingReq());
		req->acofix.dest = msg->sender;
		req->acofix.flags.POSITION_VALID = 0;
		req->acofix.flags.RANGE_VALID = 0;
		req->acofix.flags.USBL_VALID = 0;
		this->fillAcoFix(req->acofix);
		out = req;
	}
	else if (state == WAIT_PING_REPLY)
	{
		if (msg->sender == expected_id)
		{
			ROS_ERROR("Received expected id.");
			//Stop timeout waiting condition.
			sleeper.stop();
			//Handle ping reply
			//Create the PING_RESP and send navigation data
			PingResp::Ptr resp(new PingResp());
			//\todo Set only range for modems
			resp->acofix.dest = expected_id;
			resp->acofix.range.dist = msg->range;
			resp->acofix.usbl.azimuth = msg->azimuth;
			resp->acofix.usbl.elevation = msg->elevation;
			this->fillAcoFix(resp->acofix);
			out = resp;
			ROS_ERROR("Message processed.");
		}
		else
		{
			ROS_ERROR("Unexpected id.");
			//Send PING_ERROR with XCVR_RESP_WRONG
			PingError::Ptr err(new PingError());
			err->beacon_id = expected_id;
			err->status = CST::XCVR::RESP_WRONG;
			out = err;
		}
	}
	this->sendMessage(out);
	this->setState(IDLE);
}

void SeatracSim::onUSBLTimeout(const ros::TimerEvent& e)
{
	ROS_ERROR("SeatracSim: USBL timeout.");

	int state = getState();

	if (state == WAIT_PING_REPLY)
	{
		sendError<PingError>(CST::XCVR::RESP_TIMEOUT);
	}
	else if (state == WAIT_DATA_REPLY)
	{
		///
	}

	this->setState(IDLE);
}

PLUGINLIB_EXPORT_CLASS(labust::seatrac::SeatracSim, labust::seatrac::SeatracComms)

