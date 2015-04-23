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
#ifndef SEATRAC_SEATRACFACTORY_H_
#define SEATRAC_SEATRACFACTORY_H_
#include <labust/seatrac/seatrac_messages.h>
#include <boost/function.hpp>
#include <map>
#include <sstream>

namespace labust
{
	namespace seatrac
	{
		///Generic instance creator
		template <class Type>	inline SeatracMessage* createInstance(){return new Type();}


		///Factory for Seatrac message creation.
		class SeatracFactory
		{
			///Creator function callback
			typedef boost::function<SeatracMessage*(void)> Builder;
			///Creator Map
			typedef std::map<int, Builder> CID2ClassMap;
			///Name map for CID
			typedef std::map<int, std::string> NameMap;
		public:
			///Create a command message based on the supplied CID.
			static SeatracMessage::Ptr createCommand(int cid);
			///Create a response message based on the supplied CID.
			static SeatracMessage::Ptr createResponse(int cid);
			///Return the human readable name of the message given a CID.
			static const std::string& getCommandName(int cid);
			///Return the human readable name of the message given a CID.
			static const std::string& getResponseName(int cid);
			///Make the whole command message packet from a SeatracMessage type.
			static void encodePacket(const SeatracMessage::ConstPtr& msg, std::string* packet);
			///Validate and decode the data into a SeatracMessage type from the whole packet.
			static bool decodePacket(const std::string& packet, SeatracMessage::Ptr& msg);
			///Make the whole command message packet from a SeatracMessage type.
			template <class MessageType>
			static void encodePacket(const typename MessageType::ConstPtr& msg, std::string* packet)
			{
				encodePacket(boost::static_pointer_cast<SeatracMessage>(msg), packet);
			}

		protected:
			static CID2ClassMap cmdmap;
			static CID2ClassMap respmap;
			static NameMap cmdnames;
			static NameMap respnames;
		};
	}
}
/* SEATRAC_SEATRACFACTORY_H */
#endif
