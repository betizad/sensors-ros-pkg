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
#include <labust/comms/caddy/surface_handler.h>
#include <labust/comms/caddy/caddy_messages.h>
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/seatrac_definitions.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/packer.h>

#include <pluginlib/class_list_macros.h>

#include <auv_msgs/NavSts.h>
#include <std_msgs/UInt8.h>
#include <caddy_msgs/LawnmowerReq.h>

#include <string>

using namespace labust::seatrac;
using namespace labust::comms::caddy;

bool SurfaceHandler::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
    	ph.param("lawnmower_scaling", lm_scale, lm_scale);
    	ph.param("surface_delay", surface_delay, surface_delay);
 	surfacenav_pub = nh.advertise<auv_msgs::NavSts>("surface_nav",	1);
 	surfacecmd_pub = nh.advertise<std_msgs::UInt8>("surface_cmd",	1);
 	lawnreq_pub = nh.advertise<caddy_msgs::LawnmowerReq>("lawnmower_req",	1);
	return true;
}

void SurfaceHandler::operator()(const labust::seatrac::DatReceive& msg)
{
	///TODO: add handling of messages based on message ID
	///TODO: create acofix processor similar to pinger class
	/// in order to process position in-place and fuse with
	/// payload information
	SurfaceReport surf;
	if (!labust::tools::decodePackable(msg.data, &surf))
	{
		ROS_WARN("SurfaceHandler: Wrong message received from modem.");
		return;
	}

	auv_msgs::NavSts::Ptr surfnav(new auv_msgs::NavSts());
	surfnav->position.north = surf.offset_x;
	surfnav->position.east = surf.offset_y;
	surfnav->gbody_velocity.x = surf.speed;
	surfnav->orientation.yaw = labust::math::wrapRad(M_PI*surf.course/180);
	surfnav->header.stamp = ros::Time::now() - ros::Duration(surface_delay);
	surfacenav_pub.publish(surfnav);

	//Add handling of mission_cmd and lawn parameters
	if (surf.mission_cmd != last_cmd)
	{
		last_cmd = surf.mission_cmd;

		std_msgs::UInt8 cmd;
		cmd.data = surf.mission_cmd;
		surfacecmd_pub.publish(cmd);

		//The lawn mower parameters
		if ((surf.mission_cmd == LAWN_MOWER) &&
			 (surf.lawn_length != 0) &&
			 (surf.lawn_width != 0))
		{
			caddy_msgs::LawnmowerReq req;
			req.header.stamp = ros::Time::now();
			req.length = surf.lawn_length*lm_scale;
			req.width = surf.lawn_width*lm_scale;
	  	lawnreq_pub.publish(req);
		}
	}
}
