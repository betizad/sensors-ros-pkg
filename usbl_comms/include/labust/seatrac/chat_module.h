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
#ifndef USBL_COMMS_CHAT_MODULE_H
#define USBL_COMMS_CHAT_MODULE_H
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
		class ChatModule
		{
		public:
			///Main constructor
		    ChatModule():has_chat(false),predefined_chat(0),confirmed(false){};
			///Default destructor
			~ChatModule(){};

			///Listener configuration.
			bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
			{
			  text_sub = nh.subscribe("chat_text", 1, &ChatModule::onChatText, this);
			  predefined_sub = nh.subscribe("chat_predefined", 1, &ChatModule::onPredefinedChat, this);
			  return true;
			}

			///Pull the newest data in the report message the offset is the acoustic frame location
			template <class ReportMessage>
			void updateReport(ReportMessage& message)
			{
			  // Determine if it has chat messages
			  message.predefined_chat = predefined_chat;
			}

            /// Set confirmation
            void setConfirmation(bool flag)
            {
              this->confirmed = flag;
              // Do stuff
            }

		protected:
            ///Handle the textual chat messages.
            void onChatText(const std_msgs::String::ConstPtr& msg)
            {
                ROS_ERROR("Chat text not implemented.");
            }

            ///Handle the predefined chat messages.
            void onPredefinedChat(const std_msgs::Int32::ConstPtr& msg)
            {
                ROS_INFO("Received new chat: %d", predefined_chat);
                predefined_chat = msg->data;
                has_chat = true;
            }

			/// Subscription to textual messages.
			ros::Subscriber text_sub;
			/// Subscription to predefined messages
			ros::Subscriber predefined_sub;
			/// Save the predefined chat info.
			int predefined_chat;
			/// Chat indicator flag
			bool has_chat;
			/// Outgoing delivery confirmation
			bool confirmed;
		};
	}
}

/* USBL_COMMS_CHAT_MODULE_H */
#endif
