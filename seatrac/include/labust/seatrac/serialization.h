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
#ifndef SEATRAC_SEATRACSERIALIZATION_H
#define SEATRAC_SEATRACSERIALIZATION_H
#include <labust/seatrac/seatrac_messages.h>

#include <boost/serialization/level.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/static_assert.hpp>
#include <boost/crc.hpp>

namespace labust
{
	namespace seatrac
	{
		template <class Type>
		inline bool	seatrac_serialize(Type* msg, SeatracMessage::DataBuffer& out)
		{
			using boost::iostreams::back_insert_device;
			using boost::iostreams::stream;
			typedef back_insert_device<SeatracMessage::DataBuffer> smsink;
			smsink sink(out);
			stream<smsink> os(sink);
			boost::archive::binary_oarchive outSer(os, boost::archive::no_header);
			uint8_t cid = Type::CID;
			outSer << cid;
			outSer << (*msg);
		}

		template <class Type>
		inline bool seatrac_deserialize(Type* msg, const SeatracMessage::DataBuffer& in)
		{
			using boost::iostreams::array_source;
			using boost::iostreams::stream;

			if (msg->getCid() != Type::CID) return false;
			std::stringstream sin;
		  array_source source(in.data(), in.size());
		  stream<array_source> is(source);
		  uint8_t cid;
			boost::archive::binary_iarchive inSer(is, boost::archive::no_header);
			is >> cid;
			if (cid != Type::CID) return false;
			inSer >> (*msg);
		}
	}
}

/* SEATRAC_SEATRACSERIALIZATION_H */
#endif
