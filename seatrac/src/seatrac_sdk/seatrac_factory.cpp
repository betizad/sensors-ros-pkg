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
#include <labust/seatrac/seatrac_factory.h>
#include <labust/tools/StringUtilities.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/crc.hpp>
#include <sstream>

using namespace labust::seatrac;

SeatracFactory::CID2ClassMap SeatracFactory::cmdmap = {
	#include <labust/seatrac/detail/command_factory_initializer.h>
};

SeatracFactory::CID2ClassMap SeatracFactory::respmap = {
	#include <labust/seatrac/detail/response_factory_initializer.h>
};

SeatracFactory::NameMap SeatracFactory::cmdnames = {
	#include <labust/seatrac/detail/command_names.h>
};

SeatracFactory::NameMap SeatracFactory::respnames = {
	#include <labust/seatrac/detail/response_names.h>
};

SeatracMessage::Ptr SeatracFactory::createCommand(int cid)
{
	CID2ClassMap::const_iterator it = cmdmap.find(cid);
	if (it == cmdmap.end())
	{
		std::stringstream out;
		out << "SeatracFactory: no creator for command message CID = 0x";
		out << std::hex << cid <<std::endl;
		throw std::runtime_error(out.str());
	}
	return SeatracMessage::Ptr(it->second());
}

SeatracMessage::Ptr SeatracFactory::createResponse(int cid)
{
	CID2ClassMap::const_iterator it = respmap.find(cid);
	if (it == respmap.end())
	{
		std::stringstream out;
		out << "SeatracFactory: no creator for response message CID = 0x";
		out << std::hex << cid <<std::endl;
		throw std::runtime_error(out.str());
	}
	return SeatracMessage::Ptr(it->second());
}

const std::string& SeatracFactory::getCommandName(int cid)
{
	NameMap::const_iterator it = cmdnames.find(cid);
	if (it == cmdnames.end())
	{
		std::stringstream out;
		out << "SeatracFactory: no name for command message CID = 0x";
		out << std::hex << cid <<std::endl;
		throw std::runtime_error(out.str());
	}
	return it->second;
}

const std::string& SeatracFactory::getResponseName(int cid)
{
	NameMap::const_iterator it = respnames.find(cid);
	if (it == respnames.end())
	{
		std::stringstream out;
		out << "SeatracFactory: no name for response message CID = 0x";
		out << std::hex << cid <<std::endl;
		throw std::runtime_error(out.str());
	}
	return it->second;
}

void SeatracFactory::encodePacket(const SeatracMessage::ConstPtr& msg, std::string* packet)
{
	SeatracMessage::DataBuffer binary;
	using namespace boost::iostreams;
	typedef back_insert_device<SeatracMessage::DataBuffer> smsink;
	smsink sink(binary);
	stream<smsink> os(sink);
	boost::archive::binary_oarchive outser(os, boost::archive::no_header);
	uint8_t cid = msg->getCid();
	outser << cid;
	msg->pack(outser);
	os.close();

	boost::crc_16_type checksum;
	checksum.process_bytes(&binary[0], binary.size());
	uint16_t chk = checksum.checksum();
	char* pt = reinterpret_cast<char*>(&chk);
	binary.push_back(pt[0]);
	binary.push_back(pt[1]);

	std::ostringstream out("");
	if (msg->isCommand()) out<<"#"; else out<<"$";
	labust::tools::binaryToHex(binary.begin(), binary.end(), out);
	out<<'\r'<<'\n';
	packet->assign(out.str());
}

bool SeatracFactory::decodePacket(const std::string* const packet, SeatracMessage::Ptr& msg)
{
	enum {PSTART=1, PTRUNC=2, CRC_BYTES=2, MIN_SIZE=6};
	//Check size
	if (packet->size() < 6) return false;
	//Fail if the packet does not start correctly
	if ((packet->at(0) != '$') && (packet->at(0) != '#')) return false;

	//Skip '$' and '\r\n'
	SeatracMessage::DataBuffer binary;
	labust::tools::hexToBinary(packet->begin()+PSTART, packet->end()-PTRUNC, &binary);
	//Checksum
	boost::crc_16_type checksum;
	checksum.process_bytes(binary.data(), binary.size() - CRC_BYTES);
	uint16_t* chk = reinterpret_cast<uint16_t*>(&binary[binary.size() - CRC_BYTES]);

	if ((*chk) != checksum.checksum()) return false;

	if (packet->at(0) == '$')
	{
		msg = SeatracFactory::createResponse(binary[0]);
	}
	else
	{
		msg = SeatracFactory::createCommand(binary[0]);
	}

	using namespace boost::iostreams;
  array_source source(binary.data(), binary.size()-CRC_BYTES);
  stream<array_source> is(source);
	boost::archive::binary_iarchive inser(is, boost::archive::no_header);
	uint8_t cid;
	inser>>cid;
	msg->unpack(inser);

	return true;
}
