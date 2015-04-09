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
#ifndef SEATRAC_BRIDGECONTROLLER_H
#define SEATRAC_BRIDGECONTROLLER_H
#include <labust/seatrac/device_controller.h>
#include <labust/seatrac/seatrac_messages.h>

#include <std_msgs/String.h>
#include <ros/ros.h>

namespace labust
{
	namespace seatrac
	{
		/**
		 * The class implements a special bridge that re-encodes modem messages
		 * to a string topic in ROS. It is used to extend the modem simulator to
		 * hardware devices that communicate with the modem (e.g. the Tablet).
		 */
		class BridgeController : virtual public DeviceController
		{
		public:
			///Main constructor
			BridgeController();

			///Listener configuration.
			bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph);
			///Register for messages
			const RegisterMap& getRegistrations(){return registrations;}
			///Register callback for interrogation.
			void registerCallback(const SeatracComms::CallbackType& callback){this->sender = callback;};

		private:
			///Registration map
			RegisterMap registrations;
			///Sender callback
			SeatracComms::CallbackType sender;

			/**
			 * Handles the seatrac messages from the device.
			 */
			bool onMsg(const SeatracMessage::ConstPtr& msg);
			/**
			 * Handles incoming messages
			 */
			void onIncoming(const std_msgs::String::ConstPtr& msg);

			///The auto interrogation.
			//void autorun();
			///Helper function to send a single message from the queue.
			//void sendPkg();

			///Helper method for message assembly
			//SeatracMessage::Ptr makeDataCmd(const underwater_msgs::ModemTransmission::ConstPtr& msg,
			//		uint8_t msgtype);
			///Helper method for message assembly
			//SeatracMessage::Ptr makePingCmd(const underwater_msgs::ModemTransmission::ConstPtr& msg);
			///Helper method for handling returned errors
			//bool onPingErrors(const SeatracMessage::ConstPtr& msg);
			///Helper method for handling returned messages
			//bool onPingReplies(const SeatracMessage::ConstPtr& msg);
			///Helper method for unlocking the waiting condition.
			//void unlock();
			///The timeout indicator publisher
			ros::Publisher outgoing;
			///The modem transmission request subscription.
			ros::Subscriber incoming;
		};
	}
}

/* SEATRAC_USBLCONTROLLER_H */
#endif
