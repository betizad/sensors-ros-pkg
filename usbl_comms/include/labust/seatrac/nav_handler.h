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
#ifndef USBL_COMMS_NAV_HANDLER_H
#define USBL_COMMS_NAV_HANDLER_H
#include <labust/comms/caddy/caddy_messages.h>
#include <labust/math/NumberManipulation.hpp>

#include <auv_msgs/NavSts.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>

#include <Eigen/Dense>

using labust::comms::caddy::BuddyReport;
using labust::comms::caddy::SurfaceReport;
using labust::comms::caddy::DiverReport;

namespace labust
{
  namespace comms
  {
    namespace caddy
    {
      /**
       * The class implements the navigation data module. The module handles
       * publishing of navigation data for CADDY agents.
       */
      class NavHandler
      {
        enum {n=0,e,d};
      public:
        enum {SURFACE=0, DIVER, BUDDY};
        ///Main constructor
        NavHandler(){};
        ///Default destructor
        ~NavHandler(){};

        ///Listener configuration.
        bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
        {
          diverpos_pub = nh.advertise<auv_msgs::NavSts>("other_agent_diver_position_in", 1);
          divernav_pub = nh.advertise<auv_msgs::NavSts>("diver_position_in", 1);
          navall_pub = nh.advertise<auv_msgs::NavSts>("other_agent_position_in", 1);
          nav_pub = nh.advertise<auv_msgs::NavSts>("other_agent_partial_position_in", 1);
          init_pub = nh.advertise<geometry_msgs::PointStamped>("acoustic_origin_in", 1);
          return true;
        }

        ///Pull the newest data in the report message the offset is the acoustic frame location
        template <class ReportMessage>
        void updateReport(const ReportMessage& msg, const Eigen::Vector3d& offset);

      protected:
        ///Buddy speed and course navigation data publisher
        ros::Publisher nav_pub;
        ///Buddy complete navigation data publisher
        ros::Publisher navall_pub;
        ///The diver navigation info publisher
        ros::Publisher diverpos_pub;
        ///The diver orientation/etc info publisher
        ros::Publisher divernav_pub;
        /// The initialization position publisher
        ros::Publisher init_pub;
      };

      ///Buddy specialization
      template <>
      void NavHandler::updateReport<BuddyReport>(const BuddyReport& message, const Eigen::Vector3d& offset);
      ///Surface specialization
      template <>
      void NavHandler::updateReport<SurfaceReport>(const SurfaceReport& message, const Eigen::Vector3d& offset);
      ///Diver specialization
      template <>
      void NavHandler::updateReport<DiverReport>(const DiverReport& message, const Eigen::Vector3d& offset);
    }
  }
}

/* USBL_COMMS_NAV_HANDLER_H */
#endif
