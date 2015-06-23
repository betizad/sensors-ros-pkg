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
 *********************************************************************/
#include <labust/seatrac/uros_usbl_controller.h>
#include <labust/comms/uros/uros_messages.h>
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/seatrac_definitions.h>
#include <labust/seatrac/mediator.h>
#include <labust/tools/packer.h>

#include <pluginlib/class_list_macros.h>

#include <misc_msgs/RhodamineData.h>

#include <string>

using namespace labust::seatrac;
using namespace labust::comms::uros;

UROSUSBLController::UROSUSBLController():
		pinger(sender, registrations),
		id(2),
		msg_updated(false),
		ping_rate(0)
{
	registrations[DatReceive::CID].push_back(Mediator<DatReceive>::makeCallback(
			boost::bind(&UROSUSBLController::onData,this,_1)));
}

UROSUSBLController::~UROSUSBLController()
{
	run_flag = false;
	worker.join();
}

bool UROSUSBLController::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	//Init the command just in case
	message.cmd_flag = 0;

	//Lupis pinger id
	ph.param("lupis_id", id, id);
	ph.param("ping_rate", ping_rate, ping_rate);

	cmd_sub = nh.subscribe("cmd", 1,&UROSUSBLController::onCommand, this);
	nav_sub = nh.subscribe("navsts", 1, &UROSUSBLController::onEstimatedPos, this);

	adc_pub = nh.advertise<misc_msgs::RhodamineData>("rhodamine", 1);
	state_pub = nh.advertise<auv_msgs::NavSts>("rhodamine_navsts",	1);
	status_pub = nh.advertise<std_msgs::UInt8>("status",	1);

	run_flag = true;
	worker = boost::thread(boost::bind(&UROSUSBLController::run, this));

	return true;
}

void UROSUSBLController::run()
{
	ros::Rate rate((ping_rate==0)?1:ping_rate);

	while(ros::ok() && run_flag)
	{
		DatSendCmd::Ptr data(new DatSendCmd());
		data->dest = id;
		data->msg_type = AMsgType::MSG_REQU;

		boost::mutex::scoped_lock lock(message_mux);
		if (msg_updated)
		{
			SeatracMessage::DataBuffer buf;
			labust::tools::encodePackable(message,&buf);
			data->data.assign(buf.begin(),buf.end());
			msg_updated = false;
		}
		//Note: no message update sends an empty ping
		lock.unlock();

		if (!pinger.send(boost::dynamic_pointer_cast<SeatracMessage>(data), TIMEOUT))
		{
			ROS_ERROR("URSOUSBLController: Message sending failed.");
		}

		if (ping_rate) rate.sleep();
	}
}

void UROSUSBLController::onData(const labust::seatrac::DatReceive& msg)
{
	RhodamineData dat;
	if (!labust::tools::decodePackable(msg.data, &dat))
	{
		ROS_WARN("UROSUSBLController: Empty message received from modem.");
		return;
	}

	//Update the position
	latlon.convert(dat.lat, dat.lon, LLBITS_IN);

	//Publish Rhodamine data
	misc_msgs::RhodamineData::Ptr rh(new misc_msgs::RhodamineData());
	rh->position.latitude = latlon.latitude;
	rh->position.longitude = latlon.longitude;
	rh->position.altitude = -float(dat.depth) / RhodamineData::DEPTH_SC;
	rh->adc.adc = dat.adc;
	rh->adc.gain = dat.adc_gain;
	rh->header.stamp = ros::Time::now();
	adc_pub.publish(rh);

	//Publish remote position
	auv_msgs::NavSts::Ptr fix(new auv_msgs::NavSts());
	fix->header.stamp = ros::Time::now();
	fix->global_position.latitude = fix->origin.latitude = latlon.latitude;
	fix->global_position.longitude = fix->origin.longitude = latlon.longitude;
	fix->position.depth = float(dat.depth) / RhodamineData::DEPTH_SC;
	state_pub.publish(fix);

	//Publish status flag
	std_msgs::UInt8::Ptr cmd(new std_msgs::UInt8());
	cmd->data = dat.status_flag;
	status_pub.publish(cmd);
}

PLUGINLIB_EXPORT_CLASS(labust::seatrac::UROSUSBLController, labust::seatrac::DeviceController)
