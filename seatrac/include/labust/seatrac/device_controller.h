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
#ifndef SEATRAC_DEVICECONTROLLER_H
#define SEATRAC_DEVICECONTROLLER_H
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/message_listener.h>

#include <boost/shared_ptr.hpp>

namespace labust
{
	namespace seatrac
	{
		/**
		 * Base class for Seatrac device controllers.
		 * \todo Consider adding the callback member directly here (will no longer be an interface only class).
		 */
		class DeviceController : public virtual MessageListener
		{
		public:
			///Shared pointer to instances
			typedef boost::shared_ptr<DeviceController> Ptr;

			///Generic virtual destructor
			virtual ~DeviceController(){};

			///Register callback for message forwarding.
			virtual void registerCallback(const SeatracComms::CallbackType& callback){this->sender = callback;};

			///Start the device controller
			virtual void start(){};

			///Stop the device controller
			virtual void stop(){};

		protected:
			///Sender callback
			SeatracComms::CallbackType sender;
		};
	}
}

/* SEATRAC_DEVICECONTROLLER_H */
#endif
