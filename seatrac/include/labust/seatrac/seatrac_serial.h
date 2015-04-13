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
#ifndef SEATRAC_SEATRACSERIAL_H
#define SEATRAC_SEATRACSERIAL_H
#include <labust/seatrac/seatrac_comms.h>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>

#include <string>

namespace labust
{
	namespace seatrac
	{
		/**
		 * The class implements the Seatrac ASCII serial protocol.
		 * \todo Change to use the SeatracFactory methods for decoding and encoding
		 */
		class SeatracSerial : public virtual SeatracComms
		{
			enum {CID_START=1, CID_LEN=1};
			enum {DATA_START = 1, DATA_TRUNC = 3};
			enum {CRC_REM_POS = 4, CRC_BYTES = 2};

		public:
			/**
			 * Main constructor
			 */
			SeatracSerial();
			/**
			 * Generic destructor.
			 */
			virtual ~SeatracSerial();
			/**
			 * Configures the serial port based on the ROS node handle.
			 */
			virtual bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph);
			/**
			 * Iternal function for setting up the serial port.
			 * @param portName Serial port name.
			 * @param baud Serial baud rate.
			 * @return
			 */
			bool connect(const std::string& port_name, int baud);

			/**
			 * Register the message handler.
			 */
			virtual void registerCallback(const CallbackType& callback)
			{
				boost::mutex::scoped_lock l(callback_mux);
				this->callback = callback;
			};
			/**
			 * Message send
			 */
			virtual bool send(const SeatracMessage::ConstPtr& msg);

			///Resend last message
			virtual bool resend();

		private:
			/**
			 * Hepler method for binary conversion
			 * @param data In ASCII format
			 * @param binary The vector for binary data
			 * \return Returns the binary encoded data in the binary parameter.
			 */
			void convertToBinary(const std::string& data, SeatracMessage::DataBuffer& binary);
			/**
			 * Helper method for ascii conversion
			 * @param binary
			 * @param data
			 */
			void convertToAscii(const SeatracMessage::DataBuffer& binary);

			/**
			 * Handle the incoming serial data stream.
			 * @param e	Serial error indicator
			 * @param size Size of the received message.
			 */
			void onData(const boost::system::error_code& e, std::size_t size);

			///Receive startup helper function.
			void startReceive();

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
			///Muxer for the callback
			boost::mutex callback_mux;
			///Last encoded packet
			std::stringstream out;
		};
	}
}

/* SEATRAC_SEATRACSERIAL_H */
#endif
