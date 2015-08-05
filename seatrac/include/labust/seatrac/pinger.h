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
#ifndef SEATRAC_PINGER_H
#define SEATRAC_PINGER_H
#include <labust/seatrac/message_listener.h>
#include <labust/seatrac/seatrac_messages.h>

#include <boost/thread.hpp>

namespace labust
{
	namespace seatrac
	{
		/**
		 * Helper class that implements a blocking send of an acoustic transmission.
		 * The class constructor requires the sender callback and registration map to
		 * superimpose its configuration.
		 */
		class Pinger
		{
		public:
			///Main constructor
			Pinger(SeatracComms::CallbackType& sender, MessageListener::RegisterMap& registrations);
			///Default destructor
			~Pinger();

			/**
			 * Send an blocking acoustic transmission.
			 * \return Returns true if the message was acknowledged
			 */
			bool send(const SeatracMessage::ConstPtr& msg, double timeout = 2.0, bool wait_for_ack = true);

			///Returns true if a ping error occured.
			bool isError(){return is_error;}

		private:
			///Sender callback
			SeatracComms::CallbackType& sender;

			///Helper method for handling returned errors
			bool onPingErrors(const SeatracMessage::ConstPtr& msg);
			///Helper method for handling returned messages
			bool onPingReplies(const SeatracMessage::ConstPtr& msg);
			///Helper method for unlocking the waiting condition.
			void unlock();

			///Mutex for pinging condition variable
			boost::mutex ping_mux;
			///The ping lock condition variable.
			boost::condition_variable ping_condition;
			///Busy flag for in-operation indicator (default: false)
			bool is_busy;
			///The pingger error flag
			bool is_error;
		};
	}
}

/* SEATRAC_PINGER_H */
#endif
