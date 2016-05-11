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
#ifndef USBL_COMMS_CHAT_HANDLER_H
#define USBL_COMMS_CHAT_HANDLER_H
#include <labust/comms/caddy/caddy_messages.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>

namespace labust
{
	namespace seatrac
	{
		/**
		 * The class implements the chat handling module.
		 */
		class ChatHandler
		{
		public:
			///Main constructor
		    ChatHandler(){};
			///Default destructor
			~ChatHandler(){};

			///Listener configuration.
			bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
			{
			  text_pub = nh.advertise<std_msgs::String>("chat_text_in", 1);
			  predefined_pub = nh.advertise<std_msgs::Int32>("chat_predefined_in", 1);
			  return true;
			}

			///Pull the newest data in the report message the offset is the acoustic frame location
			template <class ReportMessage>
			void operator()(ReportMessage& message)
			{
			  // Determine if it has chat messages
			  std_msgs::Int32 outpc;
			  outpc.data = message.predefined_chat;
			  predefined_pub.publish(outpc);

			  std_msgs::String outstr;
			  outstr.data.assign(message.chat.begin(), message.chat.end());
			  text_pub.publish(outstr);
			}

		protected:
			/// Subscription to textual messages.
			ros::Publisher text_pub;
			/// Subscription to predefined messages
			ros::Publisher predefined_pub;
		};
	}
}

/* USBL_COMMS_CHAT_HANDLER_H */
#endif
