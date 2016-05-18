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
#ifndef USBL_COMMS_DIVER_PAYLOAD_H
#define USBL_COMMS_DIVER_PAYLOAD_H
#include <labust/comms/caddy/caddy_messages.h>

#include <caddy_msgs/DiverPayload.h>
#include <ros/ros.h>

using labust::comms::caddy::DiverReport;

namespace labust
{
	namespace seatrac
	{
		/**
		 * The class implements the dive payload handling module.
		 */
		class DiverPayload
		{
		public:
			///Main constructor
			DiverPayload():confirmed(false){};
			///Default destructor
			~DiverPayload(){};

			///Listener configuration.
			bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
			{
			  diverinfo_sub = nh.subscribe("diver_info", 1, &DiverPayload::onDiverInfo, this);
			  return true;
			}

			///Pull the newest data in the report message the offset is the acoustic frame location
			void updateReport(DiverReport& message)
			{
			  message.alarm = payload.alarm;
			  message.avg_flipper_rate = payload.average_flipper_rate;
			  message.hearth_rate = payload.hearth_rate;
			  message.breathing_rate = payload.breathing_rate;
			  message.motion_rate = payload.motion_rate;
			  message.pad_space = payload.pad_space;
			}

            /// Set confirmation
            void setConfirmation(bool flag)
            {
              this->confirmed = flag;
            }

		protected:
            ///Handle the leak warning.
            void onDiverInfo(const caddy_msgs::DiverPayload::ConstPtr& msg)
            {
                payload = *msg;
            }

			///Diver info subscription.
			ros::Subscriber diverinfo_sub;
			///Leak information
			caddy_msgs::DiverPayload payload;
			/// Delivery confirmation
			bool confirmed;
		};
	}
}

/* USBL_COMMS_BUDDY_PAYLOAD_H */
#endif
