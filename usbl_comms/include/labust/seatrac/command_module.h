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
#ifndef USBL_COMMS_COMMAND_MODULE_H
#define USBL_COMMS_COMMAND_MODULE_H
#include <labust/comms/caddy/caddy_messages.h>

#include <std_msgs/Int32.h>
#include <caddy_msgs/LawnmowerReq.h>
#include <geometry_msgs/PointStamped.h>
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
    class CommandModule
    {
      struct PayloadData
      {
        double north;
        double east;
        double width;
        double length;
      };
      enum {n=0,e,d};
    public:
      enum
      {
        NO_CHANGE = 0,
        LAWN_CMD = 1,
        GUIDE_ME = 2,
        GET_TOOL = 3,
        TAKE_PHOTO = 4,
        FAILED_CMD = 6,
        STOP = 7
       };
      ///Main constructor
      CommandModule():last_cmd(0),confirmed(false){};
      ///Default destructor
      ~CommandModule(){};

      ///Listener configuration.
      bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
      {
        cmd_sub = nh.subscribe("command_outgoing", 1, &CommandModule::onCmd, this);
        lawncmd_sub = nh.subscribe("lawnmower_req", 1, &CommandModule::onLawnCmd, this);
        takephoto_sub = nh.subscribe<geometry_msgs::PointStamped>("photo_req", 1,
            boost::bind(&CommandModule::onPointCmd, this, TAKE_PHOTO, _1));
        guideme_sub = nh.subscribe<geometry_msgs::PointStamped>("guide_target", 1,
            boost::bind(&CommandModule::onPointCmd, this, GUIDE_ME, _1));
        return true;
      }

      ///Pull the newest data in the report message the offset is the acoustic frame location
      template <class ReportMessage>
      void updateReport(ReportMessage& message, const Eigen::Vector3d& offset)
      {
        if (!confirmed)
        {
          ROS_INFO("Set message command: %d", last_cmd);
          message.command = last_cmd;
          message.north_origin = data.north - offset(n);
          message.east_origin = data.east - offset(e);
          message.lawn_length = data.length;
          message.lawn_width = data.width;
        }
        else
        {
          message.command = 0;
        }
      }
      /// Helper method for internal confirmation
      void currentStatus(uint8_t status)
      {
        if (confirmed && (status == NO_CHANGE)) return;
        if (status == FAILED_CMD) confirmed = true;
        if (status == last_cmd) confirmed = true;
      }

      /// Set confirmation
      void setConfirmation(bool flag)
      {
        this->confirmed = flag;
      }

    protected:
      ///Handle payload less commands.
      void onCmd(const std_msgs::Int32::ConstPtr& msg)
      {
        ROS_INFO("RECEIVED command: %d", msg->data);
	if (last_cmd != msg->data)
	{	
        	last_cmd = msg->data;
	        confirmed = false;
	}
      }

      ///Handle the lawn mower command.
      void onLawnCmd(const caddy_msgs::LawnmowerReq::ConstPtr& msg)
      {
        PayloadData candidate;
        candidate.north = msg->north_origin;
        candidate.east = msg->east_origin;
        candidate.length = msg->length;
        candidate.width = msg->width;

        // Check if not identical command
        //if (last_cmd != LAWN_CMD)
        //{
          last_cmd = LAWN_CMD;
          data = candidate;
          confirmed = false;
        //}
      }

      void onPointCmd(int cmd, const geometry_msgs::PointStamped::ConstPtr& msg)
      {
        PayloadData candidate;
        candidate.north = msg->point.x;
        candidate.east = msg->point.y;
        candidate.length = 0;
        candidate.width = 0;

        //if (last_cmd != cmd)
        //{
          last_cmd = cmd;
          data = candidate;
          confirmed = false;
        //}
      }

      //Checks if supplied data is identical
      bool isIdentical(const PayloadData& candidate)
      {
        return (data.north == candidate.north) &&
            (data.east == candidate.east) &&
            (data.length == candidate.length) &&
            (data.width == candidate.width);
      }

      /// General command subscription.
      ros::Subscriber cmd_sub;
      /// Lawn-mower command subscription.
      ros::Subscriber lawncmd_sub;
      /// Take a photo command subscription.
      ros::Subscriber takephoto_sub;
      /// Guide to Point command subscription.
      ros::Subscriber guideme_sub;
      /// Save last command
      int last_cmd;
      /// Lawn command payload
      PayloadData data;
      /// Confirmation of status
      bool confirmed;
    };

    //template <>
    //void CommandModule::updateReport<BuddyReport>(BuddyReport& message, const Eigen::Vector3d& offset);
  }
}

/* USBL_COMMS_COMMAND_MODULE_H */
#endif
