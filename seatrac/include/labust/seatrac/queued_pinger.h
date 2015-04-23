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
#ifndef SEATRAC_QUEUEDPINGER_H
#define SEATRAC_QUEUEDPINGER_H
#include <labust/seatrac/device_controller.h>
#include <labust/seatrac/seatrac_messages.h>

#include <underwater_msgs/ModemTransmission.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

#include <boost/thread.hpp>

#include <vector>
#include <string>
#include <queue>
#include <map>

namespace labust
{
	namespace seatrac
	{
		/**
		 * The class implements a USBL device controller that queues messages
		 * to be send to the USBL and executes a ping when the USBL is not busy.
		 * The messages can be prioritized to enhance the sending speed.
		 * The class is designed to be used for inheritance where the derived classes
		 * can then implement the message assembly and interrogation scheme without
		 * worrying about controlling the low-level part of the USBL.
		 * The device controller does not perform automatic interrogation.
		 *
		 * \todo Add callback registration for onTimeout events
		 * \todo Add failed deliveries tracking
		 */
		class QueuedPinger : virtual public DeviceController
		{
			//Map of prioritized queues
			typedef std::map<uint8_t,
					std::queue<underwater_msgs::ModemTransmission::ConstPtr> >
			MessagePriorityQueue;

		public:
			///Main constructor
			QueuedPinger();
			///Default destructor
			~QueuedPinger();

			///Listener configuration.
			bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph);
			///Register for messages
			const RegisterMap& getRegistrations(){return registrations;}
			///Register callback for interrogation.
			void registerCallback(const SeatracComms::CallbackType& callback){this->sender = callback;};

			///Enqueue a new message
			void addToQueue(const underwater_msgs::ModemTransmission::ConstPtr& msg);

			/**
			 * The function for queue processing. This function should be called explicitly.
			 * Note: this is a blocking function.
			 */
			void processQueue();

		protected:
			///Registration map
			RegisterMap registrations;
			///Sender callback
			SeatracComms::CallbackType sender;

		private:
			//Helper function for packet sending
			void sendPkg(const SeatracMessage::ConstPtr& message);

			///Helper method for message assembly
			SeatracMessage::Ptr makeDataCmd(const underwater_msgs::ModemTransmission::ConstPtr& msg,
					uint8_t msgtype);
			///Helper method for message assembly
			SeatracMessage::Ptr makePingCmd(const underwater_msgs::ModemTransmission::ConstPtr& msg);
			///Set the reply
			SeatracMessage::Ptr makeReply(const underwater_msgs::ModemTransmission::ConstPtr& msg);

			///Helper method for handling returned errors
			bool onPingErrors(const SeatracMessage::ConstPtr& msg);
			///Helper method for handling returned messages
			bool onPingReplies(const SeatracMessage::ConstPtr& msg);
			///Helper method for unlocking the waiting condition.
			void unlock();

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

			///Mutex for pinging condition variable
			boost::mutex ping_mux;
			///The ping lock condition variable.
			boost::condition_variable ping_condition;

			///The queue lock condition variable.
			boost::condition_variable queue_condition;
			///Mutex for data protection
			boost::mutex queue_mux;
			///The outgoing queue
			MessagePriorityQueue outgoing;
			///The queue processor thread
			boost::thread queue_processor;
		};
	}
}

/* SEATRAC_QUEUEDPINGER_H */
#endif
