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
#include <labust/comms/caddy/diver_handler.h>
#include <labust/comms/caddy/caddy_messages.h>
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/seatrac_definitions.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/packer.h>

#include <pluginlib/class_list_macros.h>

#include <auv_msgs/NavSts.h>

#include <string>

using namespace labust::seatrac;
using namespace labust::comms::caddy;

bool DiverHandler::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	divernav_pub = nh.advertise<auv_msgs::NavSts>("diver_nav", 1);
	return true;
}

void DiverHandler::operator()(const labust::seatrac::DatReceive& msg)
{
	DiverReport diver;
	if (!labust::tools::decodePackable(msg.data, &diver))
	{
		ROS_WARN("DiverHandler: Wrong message received from modem.");
		return;
	}

	auv_msgs::NavSts::Ptr divernav(new auv_msgs::NavSts());
	divernav->orientation.yaw = labust::math::wrapRad(M_PI*diver.heading/180);
	divernav->position.depth = diver.depth;
	divernav->header.stamp = ros::Time::now();
	divernav_pub.publish(divernav);
	//TODO Handle paddle_rate, hearth_rate and breathing_rate and command
}
