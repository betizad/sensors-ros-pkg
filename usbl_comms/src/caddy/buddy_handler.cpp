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
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/seatrac_definitions.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/packer.h>

#include <pluginlib/class_list_macros.h>

#include <auv_msgs/NavSts.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>

#include <string>

using namespace labust::seatrac;
using namespace labust::comms::caddy;

bool BuddyHandler::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
  // Navigation publishers
  divernav_pub = nh.advertise<auv_msgs::NavSts>("buddy_diver_pos", 1);
  nav_pub = nh.advertise<auv_msgs::NavSts>("buddy_pos", 1);
  partialnav_pub = nh.advertise<auv_msgs::NavSts>("buddy_partial_pos", 1);
  init_pub = nh.advertise<geometry_msgs::PointStamped>("buddy_acoustic_origin_in", 1, true);

  // Status handler
  status.configure(nh, ph);
  // Leak publishers
  leak_pub = nh.advertise<std_msgs::Bool>("buddy_leak", 1);
  battery_pub = nh.advertise<std_msgs::UInt8>("buddy_battery_status", 1);
  return true;
}

void BuddyHandler::operator()(const BuddyReport& message, const Eigen::Vector3d& offset, double delay)
{
  navHandler(message, offset, delay);
  if (message.inited)
  {
    payloadHandler(message);
    status(message, offset);
  }
}

void BuddyHandler::navHandler(const BuddyReport& message, const Eigen::Vector3d& offset, double delay)
{
  ros::Time msg_time = ros::Time::now() - ros::Duration(delay);
  if (message.inited)
  {
    ROS_INFO("Message flags: %d %d", message.has_position, message.has_diver);
    ROS_INFO("Message info: %f %f", message.north, message.east);
    auv_msgs::NavSts::Ptr nav(new auv_msgs::NavSts());
    nav->header.stamp = msg_time;
    nav->orientation.yaw = labust::math::wrapRad(M_PI*message.course/180);
    nav->gbody_velocity.x = message.speed;

    if (message.has_position)
    {
      nav->position.north = message.north + offset(n);
      nav->position.east = message.east + offset(e);
      nav->position.depth = message.depth + offset(d);
      nav->altitude = message.altitude - offset(d);

      nav_pub.publish(nav);
      partialnav_pub.publish(nav);
    }
    else
    {
      partialnav_pub.publish(nav);
    }

    //Handle diver position
    if (message.has_diver)
    {
      auv_msgs::NavSts::Ptr divernav(new auv_msgs::NavSts());
      divernav->position.north = message.diver_north;
      divernav->position.east = message.diver_east;
      divernav->header.stamp = msg_time;
      divernav_pub.publish(divernav);
    }
  }
  else
  {
    geometry_msgs::PointStamped::Ptr point(new geometry_msgs::PointStamped());
    point->header.stamp = msg_time;
    point->point.x = message.origin_lat;
    point->point.y = message.origin_lon;
    point->point.z = 0;
    init_pub.publish(point);
  }
}

void BuddyHandler::payloadHandler(const BuddyReport& message)
{
  if (message.inited)
  {
    std_msgs::Bool leak;
    leak.data = message.leak_info;
    leak_pub.publish(leak);

    std_msgs::UInt8 battery;
    battery.data = message.battery_status;
    battery_pub.publish(battery);
  }
}
