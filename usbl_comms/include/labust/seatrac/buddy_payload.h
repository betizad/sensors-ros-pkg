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
#ifndef USBL_COMMS_BUDDY_PAYLOAD_H
#define USBL_COMMS_BUDDY_PAYLOAD_H
#include <labust/comms/caddy/caddy_messages.h>
#include <labust/math/NumberManipulation.hpp>

#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <ros/ros.h>

using labust::comms::caddy::BuddyReport;

namespace labust
{
	namespace seatrac
	{
		/**
		 * The class implements the navigation data module. The module handles
		 * acquisition of navigation data for CADDY agents.
		 */
		class BuddyPayload
		{
		public:
			///Main constructor
			BuddyPayload():leak(0),battery(0){};
			///Default destructor
			~BuddyPayload(){};

			///Listener configuration.
			bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
			{
			  leak_sub = nh.subscribe("leak", 1, &BuddyPayload::onLeak, this);
			  battery_sub = nh.subscribe("battery_status", 1, &BuddyPayload::onBatteryInfo, this);
			  return true;
			}

			///Pull the newest data in the report message the offset is the acoustic frame location
			void updateReport(BuddyReport& message)
			{
			  message.leak_info = leak;
			  message.battery_status = battery;
			}

		protected:
            ///Handle the leak warning.
            void onLeak(const std_msgs::Bool::ConstPtr& msg)
            {
                leak = msg->data;
            }
            ///Handle the battery info.
            void onBatteryInfo(const std_msgs::UInt8::ConstPtr& msg)
            {
                battery = msg->data;
            }

			///Leak sensor subscription.
			ros::Subscriber leak_sub;
			///Battery status subscription.
			ros::Subscriber battery_sub;
			///Leak information
			uint8_t leak;
			///Battery information
			uint8_t battery;
		};
	}
}

/* USBL_COMMS_BUDDY_PAYLOAD_H */
#endif
