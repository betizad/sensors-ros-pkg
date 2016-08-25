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
#include <labust/seatrac/nav_handler.h>
#include <labust/math/NumberManipulation.hpp>

using labust::comms::caddy::NavHandler;

template<>
void NavHandler::updateReport<BuddyReport>(const BuddyReport& message, const Eigen::Vector3d& offset)
{
  if (message.inited)
  {
    auv_msgs::NavSts::Ptr nav(new auv_msgs::NavSts());
    nav->header.stamp = ros::Time::now() - pos_delay;
    nav->orientation.yaw = labust::math::wrapRad(M_PI*message.course/180);
    nav->gbody_velocity.x = message.speed;

    if (message.has_position)
    {
      nav->position.north = message.north + offset(n);
      nav->position.east = message.east + offset(e);
      nav->position.depth = message.depth + offset(d);
      nav->altitude = message.altitude - offset(d);

      navall_pub.publish(nav);
    }
    else
    {
      nav_pub.publish(nav);
    }

    //Handle diver position
    if (message.has_diver)
    {
      auv_msgs::NavSts::Ptr divernav(new auv_msgs::NavSts());
      divernav->position.north = message.diver_north;
      divernav->position.east = message.diver_east;
      divernav->header.stamp = nav->header.stamp;
      divernav_pub.publish(divernav);
    }
  }
  else
  {
    geometry_msgs::PointStamped::Ptr point(new geometry_msgs::PointStamped());
    point->header.stamp = ros::Time::now();
    point->point.x = message.origin_lat;
    point->point.y = message.origin_lon;
    point->point.z = 0;
    init_pub.publish(point);
  }
}

template<>
void NavHandler::updateReport<SurfaceReport>(const SurfaceReport& message, const Eigen::Vector3d& offset)
{
  auv_msgs::NavSts::Ptr nav(new auv_msgs::NavSts());
  nav->position.north = message.north + offset(n);
  nav->position.east = message.east + offset(e);

  nav->orientation.yaw = labust::math::wrapRad(M_PI*message.course/180);
  nav->gbody_velocity.x = message.speed;

  nav->header.stamp = ros::Time::now() - pos_delay;
  navall_pub.publish(nav);

  if (message.is_master && message.has_diver)
  {
    auv_msgs::NavSts::Ptr divernav(new auv_msgs::NavSts());
    divernav->position.north = message.diver_north;
    divernav->position.east = message.diver_east;
    divernav->header.stamp = nav->header.stamp;
    diverpos_pub.publish(divernav);
  }
}

template <>
void NavHandler::updateReport<DiverReport>(const DiverReport& message, const Eigen::Vector3d& offset)
{
  auv_msgs::NavSts::Ptr nav(new auv_msgs::NavSts());
  nav->orientation.yaw = labust::math::wrapRad(M_PI*message.heading/180);
  nav->position.depth = message.depth;
  nav->header.stamp = ros::Time::now();
  divernav_pub.publish(nav);
}
