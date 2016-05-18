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
#include <labust/comms/ascii6bit.h>
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/seatrac_definitions.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/packer.h>

#include <pluginlib/class_list_macros.h>

#include <auv_msgs/NavSts.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <caddy_msgs/DiverPayload.h>

#include <string>

using namespace labust::seatrac;
using namespace labust::comms::caddy;
using labust::comms::Ascii6Bit;

bool DiverHandler::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	nav_pub = nh.advertise<auv_msgs::NavSts>("diver_pos", 1);
	payload_pub = nh.advertise<caddy_msgs::DiverPayload>("diver_payload", 1);

	chat.configure(nh, ph);
	command.configure(nh, ph);
	return true;
}

void DiverHandler::operator()(const DiverReport& message, const Eigen::Vector3d& offset)
{
  navHandler(message, offset);
  payloadHandler(message);
  command(message, offset);
  chat(message);
}

// Method for handling the navigation part.
void DiverHandler::navHandler(const DiverReport& message, const Eigen::Vector3d& offset)
{
  auv_msgs::NavSts::Ptr divernav(new auv_msgs::NavSts());
  divernav->orientation.yaw = labust::math::wrapRad(M_PI*message.heading/180);
  divernav->position.depth = message.depth;
  divernav->header.stamp = ros::Time::now();
  nav_pub.publish(divernav);
}

void DiverHandler::payloadHandler(const DiverReport& message)
{
  caddy_msgs::DiverPayload payload;

  payload.alarm = message.alarms;
  payload.average_flipper_rate = message.avg_flipper_rate;
  payload.hearth_rate = message.hearth_rate;
  if (message.optional_data == 1)
  {
    payload.breathing_rate = message.breathing_rate;
    payload.motion_rate = message.motion_rate;
    payload.pad_space = message.pad_space;
  }

  payload_pub.publish(payload);
}
