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
#include <labust/comms/caddy/buddy_handler.h>
#include <labust/comms/caddy/caddy_messages.h>
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/seatrac_definitions.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/packer.h>

#include <pluginlib/class_list_macros.h>

#include <auv_msgs/NavSts.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

#include <string>

using namespace labust::seatrac;
using namespace labust::comms::caddy;

bool BuddyHandler::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	divernav_pub = nh.advertise<auv_msgs::NavSts>("diver_pos", 1);
	buddynav_pub = nh.advertise<auv_msgs::NavSts>("buddy_nav",	1);
	mission_pub = nh.advertise<std_msgs::Int32>("buddy_mission_status",	1);
	leak_pub = nh.advertise<std_msgs::Bool>("buddy_leak",	1);
	battery_pub = nh.advertise<std_msgs::UInt8>("buddy_battery_status",	1);
	return true;
}

void BuddyHandler::operator()(const labust::seatrac::DatReceive& msg)
{
	///TODO: add handling of messages based on message ID
	///TODO: create acofix processor similar to pinger class
	/// in order to process position in-place and fuse with
	/// payload information
	BuddyReport buddy;
	if (!labust::tools::decodePackable(msg.data, &buddy))
	{
		ROS_WARN("BuddyHandler: Wrong message received from modem.");
		return;
	}

	auv_msgs::NavSts::Ptr buddynav(new auv_msgs::NavSts());
	buddynav->position.north = buddy.offset_x;
	buddynav->position.east = buddy.offset_y;
	buddynav->position.depth = buddy.depth;
	buddynav->altitude = buddy.altitude;
	buddynav->orientation.yaw = labust::math::wrapRad(M_PI*buddy.course/180);
  buddynav->gbody_velocity.x = buddy.speed;
	buddynav->header.stamp = ros::Time::now();
	buddynav_pub.publish(buddynav);

	//Handle mission status, leak and battery info
	std_msgs::Int32 ms;
	ms.data = buddy.mission_status;
	mission_pub.publish(ms);

	std_msgs::Bool leak;
	leak.data = buddy.leak_info;
	leak_pub.publish(leak);

	std_msgs::UInt8 battery;
	battery.data = buddy.battery_info;
	battery_pub.publish(battery);

	//Handle diver position
	auv_msgs::NavSts::Ptr divernav(new auv_msgs::NavSts());
  divernav->position.north = buddy.diver_offset_x;
  divernav->position.east = buddy.diver_offset_y;
  divernav->header.stamp = buddynav->header.stamp;
  divernav_pub.publish(divernav);
}
