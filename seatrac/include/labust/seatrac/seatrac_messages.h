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
#ifndef SEATRAC_SEATRACMESSAGES_H_
#define SEATRAC_SEATRACMESSAGES_H_
#include <labust/seatrac/datatypes.h>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <cstdint>
#include <vector>
#include <map>
#include <sstream>

namespace labust
{
	namespace seatrac
	{
		/**
		 * The Seatrac message base to allow dynamic polymorphism.
		 */
		class SeatracMessage
		{
		public:
			///Data buffer typedef
			typedef std::vector<char> DataBuffer;
			///Smart pointer to the data buffer
			typedef boost::shared_ptr<std::stringbuf> DataBufferPtr;
			///Define a constant pointer to SeatracMessage
			typedef boost::shared_ptr<SeatracMessage const> ConstPtr;
			///Define a simple pointer to SeatracMessage
			typedef boost::shared_ptr<SeatracMessage> Ptr;

			///Generic virtual destructor.
			virtual ~SeatracMessage(){};

			///Retrieve the current message CID
			virtual int getCid() const = 0;

			///Test if the message is in reponse or command form.
			virtual bool isCommand() const = 0;

			///Pack the message into the supplied data buffer
			//virtual bool pack(SeatracMessage::DataBuffer& out) const = 0;
			virtual void pack(boost::archive::binary_oarchive& out) const = 0;

			///Unpack the message into the supplied data buffer
			//virtual bool unpack(const SeatracMessage::DataBuffer& in) = 0;
			virtual void unpack(boost::archive::binary_iarchive& in) = 0;
		};

		#include <labust/seatrac/detail/command_defs.h>
		#include <labust/seatrac/detail/response_defs.h>
	}
}
/* SEATRAC_SEATRACMESSAGES_H */
#endif
