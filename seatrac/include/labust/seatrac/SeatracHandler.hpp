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
#ifndef SEATRACHANDLER_HPP_
#define SEATRACHANDLER_HPP_
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>

#include <string>

namespace labust
{
	namespace seatrac
	{
		/**
		 * The class implements the Seatrac USBL and modem node protocol handler.
		 * \todo Implement additional messages
		 * \todo Join CID with class
		 */
		class SeaTracHandler
		{
			enum {cidStart=1, cidLen=1};
			enum {dataStart = 1, dataTrunc=3};
			enum {crcRemPos = 4};

		public:
			///Message callback type definition
			typedef boost::function<void(int, std::vector<uint8_t>&)> CallbackType;

			/**
			 * Main constructor
			 */
			SeaTracHandler();
			/**
			 * Generic destructor.
			 */
			~SeaTracHandler();
			/**
			 * Connect the handler to the com port.
			 */
			bool connect(const std::string& portName, int baud);

			/**
			 * Register message handler.
			 */
			void registerCallback(CallbackType callback){this->callback = callback;};
			/**
			 * Message send
			 */
			bool send(int cid, const std::vector<uint8_t>& binary);

			///Resend last message
			bool resend();

		private:
			///Hepler method for binary conversion
			void convertToBinary(const std::string& data, std::vector<uint8_t>& binary);
			///Helper method for ascii conversion
			void convertToAscii(const std::vector<uint8_t>& binary, std::string& data);

			/**
			 * Handle the incoming data stream.
			 */
			void onData(const boost::system::error_code& e, std::size_t size);

			 ///Serial port setup.
			bool setup_port();
			///Receive startup helper function.
			void start_receive();

			///Hardware i/o service.
			boost::asio::io_service io;
			///The serial input port
			boost::asio::serial_port port;
			///The main operation thread.
			boost::thread runner;
			///The input buffer.
			boost::asio::streambuf buffer;

			///The message callback
			CallbackType callback;
			///Last encoded packet
			std::string out;
		};
	}
}

/* SEATRACHANDLER_HPP_ */
#endif
