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
#ifndef SEATRACCID_HPP_
#define SEATRACCID_HPP_
#include <labust/preprocessor/mem_serialized_struct.hpp>
#include <boost/serialization/level.hpp>
#include <boost/static_assert.hpp>
#include <cstdint>

namespace labust
{
	namespace seatrac
	{
		namespace CID_STATUS
		{
			enum{
				///Command sent to request the current system status (AHRS, Depth, Temp, etc).
				status = 0x10,
				/**
				 * Command sent to retrieve the configuration of the status system
				 * (message content and auto-output interval).
				 */
				cfg_get,
				/**
				 * Command sent to set the configuration of the status system
				 * (message content and auto-output interval).
				 */
				cfg_set
			};
		}

		namespace CID_SYS
		{
			enum{
				///Command sent to receive a simple alive message from the beacon.
				alive = 1,
				///Command sent to receive hardware & firmware identification information.
				info,
				///Command sent to soft reboot the beacon.
				reboot,
				///Command sent to perform engineering actions.
				engineering
			};
		};

		namespace CID_PROG
		{
			enum{
				///Command sent to initialise a firmware programming sequence.
				init = 0x0D,
				///Command sent to transfer a firmware programming block.
				block,
				///Command sent to update the firmware once program transfer has completed.
				update
			};
		};

		namespace CID_XCVR
		{
		 enum{
			 /**
			  * Command sent to instruct the receiver to perform a noise
			  * analysis and report the results.
			  */
			 analyse = 0x30,
			 ///Message sent when the transceiver transmits a message.
			 tx_msg,
			 ///Message sent when the transceiver receiver encounters an error.
			 rx_err,
			 ///Message sent when the transceiver receives a message (not requiring a response).
			 rx_msg,
			 ///Message sent when the transceiver receives a request (requiring a response).
			 rx_req,
			 ///Message sent when the transceiver receives a response (to a transmitted request).
			 rx_resp,
			 /**
			  * Message sent when the transceiver is expecting a response to a request,
			  * but anerror occurs and a response can't be received.
			  */
			 rx_resp_error,
			 ///Message sent when a message has been received but not handled by the protocol stack.
			 rx_unhandled,
			 ///Message sent when a USBL signal is decoded into an angular bearing.
			 usbl,
			 /**
			  * Message sent when the transceiver gets a position/range fix
			  * on a beacon from a request/response.
			  */
			 fix
		 };
		}

		namespace CID_PING
		{
			enum{
				///Command sent to transmit a PING message.
				send = 0x40,
				///Message sent when a PING request is received.
				req,
				/**
				 * Message sent when a PING response is received,
				 * or timeout occurs, with the echo response data.
				 */
				resp,
				///Message sent when a PING response error/timeout occurs.
				error
			};
		}

		namespace CID_DATA
		{
			enum{
				///Message sent to transmit a datagram to another beacon
				send = 0x60,
				///Message generated when a beacon receives a datagram.
				receive,
				///Message generated when a datagram ACK is received back from the remote beacon.
				ack,
				///Message generated when a beacon response error/timeout occurs for ACKs.
				dat_error
			};
		}
	};
};

	/* SEATRACCID_HPP_ */
#endif
