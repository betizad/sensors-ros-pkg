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
#include <labust/seatrac/SeatracNode.hpp>
#include <labust/seatrac/SeatracCID.hpp>
#include <labust/seatrac/SeatracDefinitions.hpp>
#include <labust/seatrac/SeatracMessages.hpp>

#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float32.h>

#include <boost/bind.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

using namespace labust::seatrac;

SeaTracNode::SeaTracNode():
		ping_timeout(5),
		isBusy(false),
		autoMode(true),
		isMaster(true),
		isUsbl(true),
		onlyAck(true),
		transponderId(1),
		curTrackId(0),
		nextPingId(0)
{
	this->onInit();
}

SeaTracNode::~SeaTracNode()
{
	this->stop();
}

void SeaTracNode::start()
{
	if (isMaster && autoMode)
	{
		ROS_INFO("Starting auto-mode interrogation.");
		worker = boost::thread(boost::bind(&SeaTracNode::autorun,this));
	}
}

void SeaTracNode::stop()
{
	ROS_INFO("Stopping auto-interrogation.");
	{
		boost::mutex::scoped_lock lock(pingLock);
		this->isBusy = false;
		this->autoMode = false;
	}
	usblCondition.notify_all();
	if (worker.joinable()) worker.join();
}

void SeaTracNode::onInit()
{
	ros::NodeHandle nh, ph("~");
	std::string portName("/dev/ttyUSB0");
	int baud(115200);
	ph.param("PortName",portName,portName);
	ph.param("Baud",baud,baud);

	ros::Rate r(1);
	bool setupOk(false);
	while (!(setupOk = comms.connect(portName, baud)) && ros::ok())
	{
		ROS_ERROR("SeaTracNode::Failed to open port.");
		r.sleep();
	}

	if (setupOk)
	{
		ph.param("auto_mode",autoMode,autoMode);
		ph.param("master",isMaster,isMaster);
		///\todo Add automatic detection of capabilities
		ph.param("usbl",isUsbl,isUsbl);
		ph.param("timeout",ping_timeout,ping_timeout);
		ph.param("id", transponderId, transponderId);
		ph.param("only_ack", onlyAck, onlyAck);
		///\todo remove this hardcoded part and replace with configuration
		if (transponderId == 1) trackId.push_back(2);
		else trackId.push_back(1);

		//Register message handlers
		dispatch[CID_XCVR::fix] = boost::bind(&NavHandler::operator(), &nav,_1,_2);
		dispatch[CID_STATUS::status] = boost::bind(&StatusHandler::operator(), &stats,_1,_2);

		dataSub = nh.subscribe<underwater_msgs::ModemTransmission>("outgoing_data",
				0, &SeaTracNode::onOutgoing,this);
		dataSubBW = nh.subscribe<std_msgs::String>("outgoing_data_bw",
						0, &SeaTracNode::onOutgoingBW,this);
		opMode = nh.subscribe<std_msgs::Bool>("auto_mode",	0,
				&SeaTracNode::onAutoMode,this);
		dataPub = nh.advertise<underwater_msgs::ModemTransmission>("incoming_data",1);
		dataPubBW = nh.advertise<std_msgs::String>("incoming_data_bw",1);
		allMsg = nh.advertise<std_msgs::UInt8MultiArray>("all_msgs",1);
		usblTimeout = nh.advertise<std_msgs::Bool>("usbl_timeout",1);
		remoteDepth = nh.advertise<std_msgs::Float32>("remote_depth",1);

		this->start();

		//Register main callback
		comms.registerCallback(
				boost::bind(&SeaTracNode::incomingMsg, this, _1, _2));
	}
}

bool SeaTracNode::masterProcessor(int cid, std::vector<uint8_t>& data)
{
	bool unlock = (cid == CID_PING::error);
  unlock = unlock || (cid == CID_DATA::dat_error);


	if (cid == CID_PING::error)
	{
  	  ROS_ERROR("Error code:%d", data[0]);
	}
	if (onlyAck)
	{
		//Unlock on acknowledge
		unlock = unlock || (cid == CID_PING::resp);
	}
	else
	{
		//Unlock on data
		unlock = unlock || (cid == CID_DATA::receive);
	}

	ROS_INFO("Unlock:%d", unlock);


	return unlock;
}

bool SeaTracNode::slaveProcessor(int cid, std::vector<uint8_t>& data)
{
	//Unlock if data is sent
	bool retVal = (cid == CID_DATA::send) && (data[0] == CST::ok);

  if ((cid == CID_XCVR::rx_req) && autoMode)
	{
		sendPkg();
		retVal = false;
	}

  return retVal;
}

void SeaTracNode::incomingMsg(int cid, std::vector<uint8_t>& data)
{
	//Dispatch message
	ROS_INFO("Message CID:%x", cid);

	bool unlock(false);
	//Test if the last send was received and processed by the modem
	//Resend otherwise
	if ((cid == CID_PING::send) || (cid == CID_DATA::send))
	{
		//Check if return if OK
		switch (data[0])
		{
		case CST::ok: break;
		case CST::xcvr_busy:
			ROS_WARN("Modem busy. Resend the message.");
			comms.resend();
			break;
		default:
			ROS_ERROR("Message sending failed with error: %d", data[0]);
			unlock = true;
		}
	}

	if (isMaster)
	{
		unlock = unlock || masterProcessor(cid,data);

		//If error or successful reception allow new pings
		if (unlock)
		{
			ROS_INFO("Turnaround: %f",(ros::Time::now()-lastUSBL).toSec());
			{
				boost::mutex::scoped_lock lock(pingLock);
				isBusy = false;
			}
			usblCondition.notify_one();
		}
	}
	else
	{
		slaveProcessor(cid, data);
	}

	//Other CID processors
	///\todo Add checking that
	DispatchMap::iterator it = dispatch.find(cid);
	if (it != dispatch.end()) it->second(cid, data);

	//Publish data message separately for debugging purposes
	///\todo Export this to a separate data in/out handler with buffer capabilities ?
	if (cid == CID_DATA::receive)
	{
		std::istringstream in;
		in.rdbuf()->pubsetbuf(reinterpret_cast<char*>(data.data()), data.size());
		boost::archive::binary_iarchive inSer(in, boost::archive::no_header);

		underwater_msgs::ModemTransmission::Ptr outmodem(
				new underwater_msgs::ModemTransmission());
		outmodem->receiver = transponderId;
		outmodem->action = underwater_msgs::ModemTransmission::PAYLOAD_DATA;

		uint8_t sender;
		inSer >> sender;
		outmodem->sender = sender;
		inSer >> outmodem->payload;
		outmodem->header.stamp = ros::Time::now();
		dataPub.publish(outmodem);

		//Backward compat
		std_msgs::String::Ptr outstr(new std_msgs::String());
		//Add the 48 bits indicator
		outstr->data.push_back(48);
		outstr->data.insert(outstr->data.begin()+1,
				outmodem->payload.begin(), outmodem->payload.end());
		dataPubBW.publish(outstr);
	}

	//Extract this to another and add ID
	if (cid == CID_XCVR::rx_msg)
	{
		std::istringstream in;
		in.rdbuf()->pubsetbuf(reinterpret_cast<char*>(data.data()), data.size());
		boost::archive::binary_iarchive inSer(in, boost::archive::no_header);

		std_msgs::Float32::Ptr outremote(
						new std_msgs::Float32());

		XcvrRxMsg recMsg;
		inSer>>recMsg;
		outremote->data = recMsg.acmsg.depth;
		remoteDepth.publish(outremote);
	}

	//Publish all messages for debugging
	std_msgs::UInt8MultiArray::Ptr outarray(new std_msgs::UInt8MultiArray());
	outarray->data.push_back(cid);
	outarray->data.insert(outarray->data.end(),data.begin(),data.end());
	allMsg.publish(outarray);
}

void SeaTracNode::onAutoMode(const std_msgs::Bool::ConstPtr& mode)
{
	if (mode->data == autoMode) return;
	autoMode = mode->data;
	if (mode->data)
	{
		this->start();
	}
	else
	{
		this->stop();
	}
}

void SeaTracNode::onOutgoing(const underwater_msgs::ModemTransmission::ConstPtr& msg)
{
	boost::mutex::scoped_lock l(dataMux);
	switch(msg->action)
	{
	case underwater_msgs::ModemTransmission::RANGING:
		//Only master can do simple ranging
		if (!isMaster) return;
		//Schedule next ping id
		nextPingId = msg->receiver;
		break;
	case underwater_msgs::ModemTransmission::RANGING_DATA:
		//Fall-through since data can be sent even in slave mode
	case underwater_msgs::ModemTransmission::SEND_DATA:
		//Schedule for next message
		autoreply = msg;
		break;
	case underwater_msgs::ModemTransmission::SET_REPLY:
		//Set modem auto-send/auto-reply
		autoreply = msg;
		//Return to avoid sending the data immediately
		return;
		break;
	default:
		break;
	};

	l.unlock();

	//Only master send data
	if (!autoMode && !isBusy)
	{
		sendPkg();
	}
}

void SeaTracNode::onOutgoingBW(const std_msgs::String::ConstPtr& msg)
{
	underwater_msgs::ModemTransmission::Ptr newmsg(
			new underwater_msgs::ModemTransmission());
	//Setup message from string
	newmsg->action = (isMaster?underwater_msgs::ModemTransmission::RANGING_DATA:underwater_msgs::ModemTransmission::SET_REPLY);
	newmsg->sender = transponderId;
	newmsg->receiver = (isMaster?2:1);
	newmsg->payload.assign(msg->data.begin()+1, msg->data.end());

	{
		boost::mutex::scoped_lock l(dataMux);
		autoreply = newmsg;
	}

	//Allow only the master to send
	if (!autoMode && !isBusy && isMaster)
	{
		sendPkg();
	}
}

void SeaTracNode::sendPkg()
{
	boost::mutex::scoped_lock l(dataMux);
	// Default is to send a ping
	int cid = CID_PING::send;
	std::vector<uint8_t> data;
	data.push_back(nextPingId);

	if (autoreply != 0)
	{
		//Send data rather than a single ping
		cid = CID_DATA::send;
		DatSend msg;
		msg.destId = autoreply->receiver;
		if (isUsbl)
		{
			msg.flags = (isMaster?DatMode::ack_usbl:DatMode::no_ack_usbl);
		}
		else
		{
			///\todo is no_ack_usbl useful or possible with a modem ?
			msg.flags = (isMaster?DatMode::ack:DatMode::no_ack);
		}

		msg.payload.assign(autoreply->payload.begin(), autoreply->payload.end());

		//Serialize
		std::ostringstream out;
		boost::archive::binary_oarchive outSer(out, boost::archive::no_header);
		outSer << msg;
		std::string result = out.str();
		data.assign(result.begin(), result.end());

		//Clear the processed message
		autoreply.reset();
	}
	else
	{
		//Slaves can not send ping
		if (!isMaster) return;
		//Change to next track id only when we ping
		curTrackId = (++curTrackId)%trackId.size();
		nextPingId = trackId[curTrackId];
	}

	l.unlock();

	lastUSBL= ros::Time::now();
	comms.send(cid, data);

	///Set the device as busy.
	isBusy = true;

	//Only master blocks until the cycle is complete or timeout occurs
	if (!isMaster) return;

	boost::mutex::scoped_lock lock(pingLock);
	boost::system_time const timeout=boost::get_system_time()+boost::posix_time::seconds(ping_timeout);
	while (isBusy)
	{
		if (!usblCondition.timed_wait(lock,timeout))
		{
			ROS_WARN("USBL went into timeout.");
			std_msgs::Bool data;
			data.data = true;
			isBusy = false;
			usblTimeout.publish(data);
			break;
		}
	}
}

void SeaTracNode::autorun()
{
	//Nothing to do, don't loop
	if (trackId.size() == 0)
	{
		ROS_WARN("Not IDs to track. Auto-mode will shut down.");
		return;
	}

	while (ros::ok() && autoMode)
	{
		ROS_INFO("Sending ping.");
		sendPkg();
		//Broadcast frame
		///\todo Determine if transform broadcast is neeeded
		///\todo add transform settings in initial parameters.
		///\todo determine real transform values relative to base_link
//		geometry_msgs::TransformStamped transform;
//		transform.transform.translation.x = 0.25;
//		transform.transform.translation.y = 0;
//		transform.transform.translation.z = 0.5;
//		labust::tools::quaternionFromEulerZYX(0, 0, 0,
//				transform.transform.rotation);
//		transform.child_frame_id = "usbl_frame";
//		transform.header.frame_id = "base_link";
//		transform.header.stamp = ros::Time::now();
//		frameBroadcast.sendTransform(transform);
//		NODELET_INFO("Running.");
	}
	//NODELET_INFO("Exiting run.");
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"seatrac_node");
	SeaTracNode node;
	ros::spin();

	return 0;
}


