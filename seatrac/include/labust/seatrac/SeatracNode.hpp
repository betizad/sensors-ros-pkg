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
#ifndef SEATRACNODE_HPP_
#define SEATRACNODE_HPP_
#include <labust/seatrac/SeatracHandler.hpp>
#include <labust/seatrac/NavHandler.hpp>

#include <underwater_msgs/ModemTransmission.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/function.hpp>

#include <map>

namespace labust
{
	namespace seatrac
	{
		/**
		 * The class implements the Seatrac USBL and modem node.
		 * \todo Implement additional messages
		 * \todo Add CID type ID to structures
		 * \todo Automatic detection of local ID, USBL/Modem, etc.
		 * \todo Split the master/slave implementation into states for the State design pattern
		 */
		class SeaTracNode
		{
			typedef std::map<int, SeaTracHandler::CallbackType> DispatchMap;
		public:
			/**
			 * Main constructor
			 */
			SeaTracNode();
			///Default destructor
			~SeaTracNode();
			/**
			 * Initialize and setup controller.
			 */
			void onInit();

		private:
			/**
			 * Handles outgoing messages requests.
			 */
			void onOutgoing(const underwater_msgs::ModemTransmission::ConstPtr& msg);
			/**
			 * Handles outgoing messages requests.
			 */
			void onAutoMode(const std_msgs::Bool::ConstPtr& mode);

			/**
			 * Handle the incoming data stream.
			 */
			void onData(int type, const std::vector<int>& payload);

			///Function handler
			void incomingMsg(int cid, std::vector<uint8_t>& data);

			///Automatic interrogation runner
			void autorun();
			///Stop the interrogation
			void stop();

			///Handler map
			DispatchMap dispatch;
			///Seatrack comms handler
			SeaTracHandler comms;
			///Navigation handler
			NavHandler nav;

			///Device status handler
			//StatusHandler status;
			///Device navigation handler
			//NavHandler nav;
			///Master processor of data
			bool masterProcessor(int cid, std::vector<uint8_t>& data);
			///Slave processor of data
			bool slaveProcessor(int cid, std::vector<uint8_t>& data);

			/**
			 * Send one USBL encoded package.
			 */
			void sendPkg();
			/**
			 * Ping timeout.
			 */
			double ping_timeout;
			/**
			 * Usbl message sent time.
			 */
			ros::Time lastUSBL;

			///Auto-reply payload
			underwater_msgs::ModemTransmission::ConstPtr autoreply;
			///The data mutex.
			boost::mutex dataMux;

			/// The USBL busy indicator flag.
			bool isBusy;
			/// The ping lock condition variable.
			boost::condition_variable usblCondition;
			/// The ping lock mutex
			boost::mutex pingLock;


			/**
			 * The worker thread.
			 */
			boost::thread worker;

			///Payload data
			ros::Publisher dataPub;
			///The timeout publisher.
			ros::Publisher usblTimeout;
			///The debug publisher
			ros::Publisher allMsg;
			///The modem transmission request subscription.
			ros::Subscriber dataSub;
			///The modem operation mode.
			ros::Subscriber opMode;
			///Auto interrogate mode.
			bool autoMode;
			///Is master or slave flag
			bool isMaster;
			///Flag to differentiate between USBL and modem
			bool isUsbl;
			///Flag to indicate wheather to expect a reply or just acknowledge
			bool onlyAck;
			///Vector of transponders
			std::vector<int> trackId;
			///Current tracked ID or interogation id.
			int curTrackId;
			///Next ID to ping
			int nextPingId;
			///The local transponder ID
			int transponderId;
		};
	}
}

/* SEATRACNODE_HPP_ */
#endif
