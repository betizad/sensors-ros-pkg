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
#ifndef SEATRACMESSAGES_HPP_
#define SEATRACMESSAGES_HPP_
#include <labust/preprocessor/mem_serialized_struct.hpp>
#include <boost/serialization/level.hpp>
#include <labust/seatrac/detail/vector_override.hpp>
#include <boost/static_assert.hpp>
#include <cstdint>

namespace labust
{
	namespace seatrac
	{

		struct StatusBits
		{
			bool ENVIRONMENT :1;
			bool ATTITUDE :1;
			bool MAG_CAL :1;
			bool ACC_CAL :1;
			bool AHRS_RAW_DATA :1;
			bool AHRS_COMP_DATA :1;
			bool reserved2 :1;
			bool reserved :1;
		};
		BOOST_STATIC_ASSERT(sizeof(StatusBits) == 1 && ("STATUS_BITS_T structure is assumed as size 1 byte."));

		struct AcoFixBits
		{
			bool RANGE_VALID:1;
			bool USBL_VALID :1;
			bool POSITION_VALID :1;
			bool POSITION_ENHANCED :1;
			bool POSITION_FLT_ERROR :1;
			bool reserved3 :1;
			bool reserved2 :1;
			bool reserved :1;
		};
		BOOST_STATIC_ASSERT(sizeof(AcoFixBits) == 1 && ("AcoFixBits structure is assumed as size 1 byte."));
	}
};

///Define status bits as primitve type for easier deserialization
BOOST_CLASS_IMPLEMENTATION(labust::seatrac::StatusBits, boost::serialization::primitive_type);
BOOST_CLASS_IMPLEMENTATION(labust::seatrac::AcoFixBits, boost::serialization::primitive_type);

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(seatrac),StatusHeader,
		(StatusBits, status_bits)
		(uint64_t, timestamp))

typedef std::vector<uint8_t> PayloadType;
typedef std::vector<int16_t> USBLRSSIVec;

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(seatrac), DatSend,
		(uint8_t, destId)
		(uint8_t, flags)
		(PayloadType, payload))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(seatrac), DatReceive,
		(uint8_t, srcId)
		(PayloadType, payload))


typedef int16_t vec3si[3];
typedef int32_t vec3i[3];
PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(seatrac), XcvrFix,
		(uint64_t, timestamp)
		(uint8_t, beaconId)
		(vec3si, attitude1)
		(uint16_t, vos)
		(uint8_t, range_valid)
		(uint32_t, range_count)
		(int32_t, range_time)
		(uint32_t, range_dist)
		(uint16_t, depth_local)
		(uint8_t, depth_valid)
		(uint16_t, depth_remote)
		(uint8_t, signal_valid)
		(int16_t, signal_azimuth)
		(int16_t, signal_elevation)
		(uint8_t, position_valid)
		(vec3i, position))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(seatrac), AcousticMsg,
		(uint8_t, dest)
		(int8_t, src)
		(uint8_t, type)
		(uint16_t, depth)
		(uint8_t, payloadId)
		(PayloadType, payload))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(seatrac), XcvrRxMsg,
		(AcousticMsg, acmsg)
		(int16_t, rssi))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(seatrac), EnvStatus,
		(uint16_t, supply)
		(int16_t, temp)
		(int32_t, pressure)
		(int32_t, depth)
		(uint16_t, vos))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(seatrac), MagCalibrationStatus,
		(uint8_t, cal_buf)
		(uint8_t, cal_valid)
		(uint32_t, cal_age)
		(uint8_t, cal_fit))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(seatrac), AccCalibrationStatus,
		(vec3si, lim_min)
		(vec3si, lim_max))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(seatrac), AHRSData,
		(vec3si, acc)
		(vec3si, mag)
		(vec3si, gyro))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(seatrac), AcoFixHeader,
		(uint8_t, dest)
		(uint8_t, src)
		(AcoFixBits, flags)
		(uint8_t, msg_type)
		(vec3si, attitude)
		(uint16_t, depth_local)
		(uint16_t, vos)
		(int16_t, rssi))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(seatrac), AcoFixRange,
		(uint32_t, range_count)
		(int32_t, range_time)
		(uint16_t, range_dist))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(seatrac), AcoFixUSBL,
		(USBLRSSIVec, usbl_rssi)
		(int16_t, usbl_azimuth)
		(int16_t, usbl_elevation)
		(int16_t, usbl_fit_error))

namespace labust
{
	namespace seatrac
	{
		struct AcoFix
		{
			AcoFixHeader header;
			AcoFixRange range_data;
			AcoFixUSBL usbl_data;
			vec3si position;
		};
	};
};

namespace boost
{
	namespace serialization
	{
		template<class Archive>
	  void serialize(Archive & ar, labust::seatrac::AcoFix & object, const unsigned int version)
	  {
			object.header.dest = 0;
			ar & object.header;
			labust::seatrac::AcoFixBits flags = object.header.flags;
			if (flags.RANGE_VALID) ar & object.range_data;
			if (flags.USBL_VALID) ar & object.usbl_data;
			if (flags.POSITION_VALID) ar & object.position;
	  }
	}
}

BOOST_CLASS_IMPLEMENTATION(labust::seatrac::AcoFix, boost::serialization::object_serializable)

/* SEATRACMESSAGES_HPP_ */
#endif
