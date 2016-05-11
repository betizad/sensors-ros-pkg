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
#ifndef USBL_COMMS_STATUS_HANDLER_H
#define USBL_COMMS_STATUS_HANDLER_H
#include <labust/comms/caddy/caddy_messages.h>

#include <std_msgs/Int32.h>
#include <caddy_msgs/LawnmowerReq.h>
#include <ros/ros.h>

#include <Eigen/Dense>

using labust::comms::caddy::BuddyReport;

namespace labust
{
  namespace seatrac
  {
    /**
     * The class implements the command handling module.
     */
    class StatusHandler
    {
      enum
      {
        NO_CHANGE = 0,
        LAWN_CMD = 1,
        FAILED_CMD = 14
      };
      enum {n=0,e,d};
    public:
      ///Main constructor
      StatusHandler(const std::string& user, bool is_command = false):
        is_command(is_command),
        user(user){};
      ///Default destructor
      ~StatusHandler(){};

      ///Listener configuration.
      bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
      {
        std::string prefix(is_command?user + "_command_":user + "_status_");
        status_pub = nh.advertise<std_msgs::Int32>(prefix + "in", 1);
        lawncmd_pub = nh.advertise<caddy_msgs::LawnmowerReq>(prefix + "lawnmower_req", 1);
        return true;
      }

      ///Pull the newest data in the report message the offset is the acoustic frame location
      template <class ReportMessage>
      void operator()(const ReportMessage& message, const Eigen::Vector3d& offset)
      {
        std_msgs::Int32 cmdout;
        ROS_INFO("Received command:%d", message.command);
        cmdout.data = message.command;
        status_pub.publish(cmdout);

        int cmd = message.command;
        if (cmd == LAWN_CMD)
        {
          caddy_msgs::LawnmowerReq req;
          req.length = message.lawn_length;
          req.width = message.lawn_width;
          req.north_origin = message.north_origin + offset(n);
          req.east_origin = message.east_origin + offset(e);
          req.header.stamp = ros::Time::now();
          lawncmd_pub.publish(req);
        }
      }

    protected:
      /// General command subscription.
      ros::Publisher status_pub;
      /// Lawn-mower command subscription.
      ros::Publisher lawncmd_pub;
      /// The status/command topic switcher
      bool is_command;
      /// The user topic swithcer.
      std::string user;
    };

    template <>
    void StatusHandler::operator()<BuddyReport>(const BuddyReport& message, const Eigen::Vector3d& offset);

  }
}

/* USBL_COMMS_STATUS_HANDLER_H */
#endif
