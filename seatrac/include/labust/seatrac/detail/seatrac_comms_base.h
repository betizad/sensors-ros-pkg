/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, LABUST, UNIZG-FER
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
 *
 *  Created by: Đula Nađ
 *********************************************************************/
#ifndef SEATRAC_SEATRACCOMMSBASE_H
#define SEATRAC_SEATRACCOMMSBASE_H
#include <labust/seatrac/seatrac_messages.h>

#include <vector>
#include <stdexcept>

namespace labust
{
	namespace seatrac
	{
		namespace detail
		{
			/**
			 * The interface defines the communication layer for the SeaTrac devices.
			 * The following communication layer are expected:
			 * 	1. Serial device
			 * 	2. Ethernet device
			 * 	3. Simulated device
			 */
			template <class Configurator>
			class SeatracCommsBase : public virtual Configurator
			{
			public:
				///Message callback type definition
				typedef boost::function<void(const SeatracMessage::ConstPtr&)> CallbackType;
				///Pointer to a SeatracComms instance
				typedef boost::shared_ptr<SeatracCommsBase<Configurator> > Ptr;

				/**
				 * Generic destructor.
				 */
				virtual ~SeatracCommsBase(){};

				/**
				 * Register message handler.
				 */
				virtual void registerCallback(const CallbackType& callback) = 0;
				/**
				 * Send the binary encoded Seatrac message to the communication device.
				 * @param binary The binary encoded message as a vector.
				 * \return True for success, false otherwise.
				 */
				virtual bool send(const SeatracMessage::ConstPtr& msg) = 0;

				/**
				 * Resends the last message.
				 * \return True if success, false otherwise.
				 */
				virtual bool resend() = 0;
			};
		}
	}
}

/* SEATRAC_SEATRACCOMMSBASE_H */
#endif
