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
#include <labust/seatrac/uros_modem_controller.h>
#include <labust/comms/uros/uros_messages.h>
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/seatrac_definitions.h>
#include <labust/seatrac/mediator.h>
#include <labust/tools/packer.h>

#include <pluginlib/class_list_macros.h>

#include <std_msgs/UInt8.h>
#include <sensor_msgs/NavSatFix.h>

#include <string>

using namespace labust::seatrac;
using namespace labust::comms::uros;

UROSModemController::UROSModemController():
		comms_timeout(300),
		avg(false),
		empty_reply(false)
{
	registrations[DatReceive::CID].push_back(Mediator<DatReceive>::makeCallback(
			boost::bind(&UROSModemController::onData,this,_1)));
	registrations[PingReq::CID].push_back(Mediator<PingReq>::makeCallback(
			boost::bind(&UROSModemController::onPing,this,_1)));
}

UROSModemController::~UROSModemController()
{
	nocomms.stop();
}

bool UROSModemController::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	adc_sub = nh.subscribe("rhodamine", 1, &UROSModemController::onAdc,this);
	state_sub = nh.subscribe("position",	1, &UROSModemController::onNavSts,this);
	rhodamine_avg = nh.subscribe<std_msgs::Bool>("autosStartFlag", 1, &UROSModemController::onAvg, this);
	cmd_pub = nh.advertise<std_msgs::UInt8>("cmd",1);
	nav_pub = nh.advertise<sensor_msgs::NavSatFix>("acoustic_fix",1);

	ph.param("comms_timeout", comms_timeout, comms_timeout);

	//Create timer
	nocomms = nh.createTimer(ros::Duration(comms_timeout),
			&UROSModemController::onTimeout, this,
			true, false);

	return true;
}

void UROSModemController::onAdc(const misc_msgs::RhodamineAdc::ConstPtr& msg)
{
//	RhodamineData out;
	if(max_rhodamine.adc <= msg->adc) /*** Comparison for fixed gain ***/
	{
		max_rhodamine.adc = msg->adc;
		max_rhodamine.adc_gain = uint8_t(std::log10(msg->gain));
		max_rhodamine.depth = position.position.depth * RhodamineData::DEPTH_SC;
		labust::tools::LatLon2Bits conv;
		conv.convert(position.global_position.latitude,
				position.global_position.longitude, LLBITS_OUT);
		max_rhodamine.lat = conv.lat;
		max_rhodamine.lon = conv.lon;
	}
//	out.adc = msg->adc;
//	out.adc_gain = uint8_t(std::log10(msg->gain));
//	out.depth = position.position.depth * RhodamineData::DEPTH_SC;
//	labust::tools::LatLon2Bits conv;
//	conv.convert(position.global_position.latitude,
//			position.global_position.longitude, llbits);
//	out.lat = conv.lat;
//	out.lon = conv.lon;

	max_rhodamine.status_flag = avg;

	//Do not pack the message if no reply is required
	if (empty_reply) return;

	DatQueueClearCmd::Ptr clr(new DatQueueClearCmd());
	DatQueueSetCmd::Ptr cmd(new DatQueueSetCmd());
	cmd->dest = labust::seatrac::BEACON_ALL;
	std::vector<char> binary;
//	labust::tools::encodePackable(out, &binary);
	labust::tools::encodePackable(max_rhodamine, &binary);
	cmd->data.assign(binary.begin(),binary.end());

	if (!sender.empty())
	{
		sender(clr);
		if (!empty_reply) sender(cmd);
	}
}

void UROSModemController::onData(const labust::seatrac::DatReceive& msg)
{
	///Reset timer
	boost::mutex::scoped_lock l(timer_mux);
	nocomms.stop();
	nocomms.setPeriod(ros::Duration(comms_timeout));
	nocomms.start();

	/// Reset max rhodamine value in every communication cycle
	max_rhodamine.adc = 0;

	LupisUpdate dat;
	if ((msg.data.size() == 0) || !labust::tools::decodePackable(msg.data, &dat))
	{
		ROS_WARN("UROSUSBLController: Empty message received from modem.");
		return;
	}

	//Update the position
	latlon.convert(dat.lat, dat.lon, LLBITS_IN);

	//Process LUPIS update
	//Get lat-lon and publish NavSatFix ?
	sensor_msgs::NavSatFix::Ptr fix(new sensor_msgs::NavSatFix());
	fix->header.stamp = ros::Time::now();
	fix->latitude = latlon.latitude;
	fix->longitude = latlon.longitude;
	nav_pub.publish(fix);
	//Preprocess command flag
	if (dat.cmd_flag == 15) empty_reply = true;
	if (dat.cmd_flag == 14) empty_reply = false;
	//Publish command flag
	std_msgs::UInt8::Ptr cmd(new std_msgs::UInt8());
	cmd->data = dat.cmd_flag;
	cmd_pub.publish(cmd);
}

void UROSModemController::onTimeout(const ros::TimerEvent& e)
{
	//Publish command flag
	std_msgs::UInt8::Ptr cmd(new std_msgs::UInt8());
	//Request abort
	enum {it_cnt=10};
	ros::Rate r(2);
	for(int i=0; i<it_cnt; ++i)
	{
		ROS_ERROR("Timeout on acoustic comms.");
		cmd->data = 0x01;
		cmd_pub.publish(cmd);
		r.sleep();
	}
}

void UROSModemController::onPing(const labust::seatrac::PingReq& data)
{
	boost::mutex::scoped_lock l(timer_mux);
	ROS_ERROR("On Ping request.");
	nocomms.stop();
	nocomms.setPeriod(ros::Duration(comms_timeout));
	nocomms.start();
}

PLUGINLIB_EXPORT_CLASS(labust::seatrac::UROSModemController, labust::seatrac::DeviceController)
