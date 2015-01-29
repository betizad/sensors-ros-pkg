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
#ifndef SEATRAC_USBLCONTROLLER_H
#define SEATRAC_USBLCONTROLLER_H
#include <labust/seatrac/device_controller.h>
#include <labust/seatrac/seatrac_messages.h>

#include <underwater_msgs/ModemTransmission.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

#include <boost/thread.hpp>

#include <vector>
#include <string>
#include <queue>

namespace labust
{
	namespace seatrac
	{
		/**
		 * The class implements the status publisher and decoder.
		 */
		class USBLController : virtual public DeviceController
		{
			typedef std::queue<SeatracMessage::Ptr> MessageQueue;

		public:
			///Main constructor
			USBLController();
			///Default destructor
			~USBLController();

			///Listener configuration.
			bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph);
			///Register for messages
			const RegisterMap& getRegistrations(){return registrations;}
			///Register callback for interrogation.
			void registerCallback(const SeatracComms::CallbackType& callback){this->sender = callback;};

			///Start the controller
			void start();
			///Stop the controller
			void stop();

		private:
			///Registration map
			RegisterMap registrations;
			///Sender callback
			SeatracComms::CallbackType sender;

			/**
			 * Handles outgoing messages requests.
			 */
			void onOutgoing(const underwater_msgs::ModemTransmission::ConstPtr& msg);
			/**
			 * Handles automatic/manual mode option.
			 */
			void onAutoMode(const std_msgs::Bool::ConstPtr& mode);

			///The auto interrogation.
			void autorun();
			///Helper function to send a single message from the queue.
			void sendPkg();

			///Helper method for message assembly
			SeatracMessage::Ptr makeDataCmd(const underwater_msgs::ModemTransmission::ConstPtr& msg,
					uint8_t msgtype);
			///Helper method for message assembly
			SeatracMessage::Ptr makePingCmd(const underwater_msgs::ModemTransmission::ConstPtr& msg);
			///Helper method for handling returned errors
			bool onPingErrors(const SeatracMessage::ConstPtr& msg);
			///Helper method for handling returned messages
			bool onPingReplies(const SeatracMessage::ConstPtr& msg);
			///Helper method for unlocking the waiting condition.
			void unlock();
			///The timeout indicator publisher
			ros::Publisher usbl_timeout;
			///The modem transmission request subscription.
			ros::Subscriber outSub;
			///Automatic mode flag subscriber
			ros::Subscriber opMode;
			///Configuration service

			///Automatic mode configuration (default: false)
			bool auto_mode;
			///Flag for enhanced USBL requests (default: false)
			bool enhanced_usbl;
			///Flag for enhanced data sends - ill add info with data (default: false)
			bool enhanced_data;
			///Busy flag for in-operation indicator (default: false)
			bool is_busy;
			/**
			 * The maximum timeout to wait for ping (default: 2.0).
			 * The timeout is increased if data is sent by the amount of
			 * data bytes * 100ms.
			 */
			double timeout;
			/**
			 * The interrogation ID list (default: []). This uses a
			 * vector which means that [1,2,2] is a valid interrogation
			 * sequence.
			 */
			std::vector<int> transponders;
			///The nextId to ping (default: 0)
			int nextId;
			///Mutex for data protection
			boost::mutex data_mux;
			///Mutex for pinging condition variable
			boost::mutex ping_mux;
			///The ping lock condition variable.
			boost::condition_variable ping_condition;
			///Worker thread for auto-interrogation
			boost::thread worker;
			///The outgoing queue
			MessageQueue outgoing;
		};
	}
}

/* SEATRAC_USBLCONTROLLER_H */
#endif
