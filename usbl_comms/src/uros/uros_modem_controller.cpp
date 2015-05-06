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

UROSModemController::UROSModemController()
{
	registrations[DatReceive::CID].push_back(Mediator<DatReceive>::makeCallback(
			boost::bind(&UROSModemController::onData,this,_1)));
}

UROSModemController::~UROSModemController(){}

bool UROSModemController::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	adc_sub = nh.subscribe("rhodamine", 1, &UROSModemController::onAdc,this);
	state_sub = nh.subscribe("position",	1, &UROSModemController::onNavSts,this);
	cmd_pub = nh.advertise<std_msgs::UInt8>("cmd",1);
	nav_pub = nh.advertise<sensor_msgs::NavSatFix>("acoustic_fix",1);

	return true;
}

void UROSModemController::onAdc(const misc_msgs::RhodamineAdc::ConstPtr& msg)
{
	RhodamineData out;
	out.adc = msg->adc;
	out.adc_gain = uint8_t(std::log10(msg->gain));
	out.depth = position.position.depth * RhodamineData::DEPTH_SC;
	labust::tools::LatLon2Bits conv;
	conv.convert(position.global_position.latitude,
			position.global_position.longitude, llbits);
	out.lat = conv.lat;
	out.lon = conv.lon;


	DatQueueClearCmd::Ptr clr(new DatQueueClearCmd());
	DatQueueSetCmd::Ptr cmd(new DatQueueSetCmd());
	cmd->dest = labust::seatrac::BEACON_ALL;
	std::vector<char> binary;
	labust::tools::encodePackable(out, &binary);
	cmd->data.assign(binary.begin(),binary.end());

	if (!sender.empty())
	{
		sender(clr);
		sender(cmd);
	}
}

void UROSModemController::onData(const labust::seatrac::DatReceive& msg)
{
	LupisUpdate dat;
	if (!labust::tools::decodePackable(msg.data, &dat))
	{
		ROS_WARN("UROSUSBLController: Empty message received from modem.");
		return;
	}

	//Update the position
	{
		boost::mutex::scoped_lock l(position_mux);
		latlon.setInitLatLon(position.origin.latitude,
				position.origin.longitude);
	}
	latlon.convert(dat.lat, dat.lon, llbits);

	//Process LUPIS update
	//Get lat-lon and publish NavSatFix ?
	sensor_msgs::NavSatFix::Ptr fix(new sensor_msgs::NavSatFix());
	fix->header.stamp = ros::Time::now();
	fix->latitude = latlon.latitude;
	fix->longitude = latlon.longitude;
	nav_pub.publish(fix);
	//Publish command flag
	std_msgs::UInt8::Ptr cmd(new std_msgs::UInt8());
	cmd->data = dat.cmd_flag;
	cmd_pub.publish(cmd);
}

PLUGINLIB_EXPORT_CLASS(labust::seatrac::UROSModemController, labust::seatrac::DeviceController)
