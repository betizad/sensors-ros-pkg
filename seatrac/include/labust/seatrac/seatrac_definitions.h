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
#ifndef SEATRAC_SEATRACDEFINITIONS_H
#define SEATRAC_SEATRACDEFINITIONS_H

namespace labust
{
	namespace seatrac
	{
		namespace AMsgType
		{
			enum {
				/**
				 * Indicates an acoustic message is sent One-Way, and does not require a response.
				 * One-Way messages may also be broadcast to all beacons if required.
				 * No USBL information is sent.
				 */
				MSG_OWAY = 0,
				/**
				 * Indicates an acoustic message is sent One-Way, and does not require a response.
				 * One-Way messages may also be broadcast to all
				 * beacons if required. Additionally, the message is sent with USBL
				 * acoustic information allowing an incoming bearing to be determined by USBL receivers,
				 * although range cannot be provided.
				 */
				MSG_OWAYU,
				/**
				 * Indicates an acoustic message is sent as a Request type. This requires the
				 * receiver to generate and return a Response (MSG_RESP) message.
				 * No USBL information is requested.
				 */
				MSG_REQ,
				/**
				 * Indicate an acoustic message is sent as a Response to a previous Request message
				 * (MSG_REQ). No USBL information is returned.
				 */
				MSG_RESP,
				/**
				 * Indicates an acoustic message is sent as a Request type. This requires the receiver to
				 * generate and return a Response (MSG_RESP)	message. Additionally, the Response message
				 * should be returned with USBL acoustic information allowing a position fix to be computed
				 * by USBL receivers through the range and incoming signal angle.
				 */
				MSG_REQU,
				/**
				 * Indicate an acoustic message is sent as a Response to a previous Request message
				 * (MSG_REQ). Additionally, the message is sent with USBL	acoustic information
				 * allowing the position of the sender to be determined through the range and incoming
				 * signal angle.
				 */
				MSG_RESPU,
				/**
				 * Indicates an acoustic message is sent as a Request type. This requires the
				 * receiver to generate and return a Response (MSG_RESP) message.	Additionally,
				 * the Response message should be	returned with extended Depth and USBL	acoustic
				 * information allowing a more accurate position fix to be computed by USBL
				 * receivers through the range, remote depth and incoming signal angle.
				 */
				MSG_REQX,
				/**
				 * Indicate an acoustic message is sent as a Response to a previous Request
				 * message (MSG_REQ). Additionally, the message is sent with extended depth
				 * and USBL acoustic information allowing a more accurate position of the
				 * sender to be determined through the range, remote depth and incoming
				 * signal	angle.
				 */
				MSG_RESPX,
			};
		};

		namespace APayloadType
		{
			enum{
				/**
				 * Specified an acoustic message payload should be interpreted by the
				 * PING protocol handler.	PING messages provide the simplest (and quickest)
				 * method of validating the presence of a beacon, and determining its position.
				 */
				pload_ping = 0,
				/**
				 * Specified an acoustic message payload should	be interpreted by the
				 * ECHO protocol handler.	ECHO messages allow the function and reliability
				 * of a beacon to be tested, by requesting the payload contents of the
				 * message be returned back to the sender.
				 */
				pload_echo,
				/**
				 * Specified an acoustic message payload should be interpreted by the
				 * NAV (Navigation)	protocol handler.	NAV messages allow tracking and
				 * navigation systems to be built that use enhanced	positioning and
				 * allow remote parameters of	beacons (such as heading, attitude, water
				 * temperature etc) to be queried.
				 */
				pload_nav,
				/**
				 * Specified an acoustic message payload should be interpreted by
				 * the DAT (Datagram) protocol handler.	DAT messages for the
				 * simplest method of data exchange between beacons, and provide
				 * a method of acknowledging data reception.
				 */
				pload_dat,
				/**
				 * Specified an acoustic message payload should be interpreted
				 * by the DEX (Data Exchange)	protocol handler. DEX messages
				 * implement a protocol that allows robust bi-directional
				 * socket based data exchange with timeouts, acknowledgments
				 * and retry schemes.
				 */
				pload_dex
			};
		}

		namespace DatMode
		{
			enum{
				/**
				 *  Data is sent one-way with no USBL signal request
				 *  (AMsgType is msg_oway), and no response acknowledgment
				 *  is required. As no USBL signal or response is used, no
				 *  ranging or position fix will be available on receipt on
				 *  this message.
				 */
				no_ack = 0,
				/**
				 * Data is sent one-way with a USBL signal request (AMsgType is MSG_OWAYU),
				 * but no response acknowledgment is required. As a USBL signal is transmitted
				 * with the data, the remote beacon can determine incoming signal angle,
				 * but not range or position.
				 */
				no_ack_usbl,
				/**
				 * Data is sent as a request message where the acknowledgment will be the
				 * response, but no	USBL response is required (AMsgType is MSG_RESP).
				 * As no USBL signal information is transmitted, but a response is returned,
				 * ranging information will be available for the remote	beacon.
				 */
				ack,
				/**
				 * Data is sent as a request message where the acknowledgment will be the
				 * response, and a USBL signal is required (AMsgType is	MSG_RESPU).	As both
				 * USBL signal information and a response is returned by the remote beacon,
				 * positioning and range information for the remote beacon will be available
				 * to the sender.
				 */
				ack_usbl
			};
		};

		namespace CST
		{
			enum{
				OK = 0
			};

			namespace XCVR
			{
				enum{
					BUSY = 0x30,
					RESP_TIMEOUT = 0x34,
					RESP_WRONG = 0x36
				};

			};
		}

		enum{
			///Data broadcast address
			beacon_all = 0
		};

	}
}

/* SEATRAC_SEATRACDEFINITIONS_H */
#endif
