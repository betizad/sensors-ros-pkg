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
#ifndef SEATRAC_SEATRACCORE_H
#define SEATRAC_SEATRACCORE_H
#include <labust/seatrac/seatrac_comms.h>
#include <labust/seatrac/device_controller.h>
#include <labust/seatrac/message_listener.h>
#include <pluginlib/class_loader.h>

#include <list>
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
		 * \todo Remove backward capability.
		 * \todo Optimization for ARM/UDOO ?
		 */
		class SeatracCore
		{
			typedef std::list<SeatracComms::CallbackType> CallbackList;
			typedef std::map<int, CallbackList> CallbackMap;

		public:
			///Main constructor
			SeatracCore();
			///Default destructor
			~SeatracCore();
			///Initialize and setup the Seatrac core.
			void onInit();

		private:
			///Helper function for plugin setup
			void setupPlugins(ros::NodeHandle& nh, ros::NodeHandle& ph);
			///Function handler
			bool incomingMsg(const SeatracMessage::ConstPtr& msg);
			///Function handler
			bool outgoingMsg(const SeatracMessage::ConstPtr& msg);
			///Helper function for registrations
			void addRegistrationMap(const MessageListener::RegisterMap& lmap);

			//Communication layer
			///Comms loader
			pluginlib::ClassLoader<SeatracComms> comms_loader;
			///Comms handle
			SeatracComms::Ptr comms;

			//Controller and listener layer
			///Comms loader
			pluginlib::ClassLoader<DeviceController> control_loader;
			///Comms handle
			DeviceController::Ptr controller;

			///Listeners loader
			pluginlib::ClassLoader<MessageListener> listener_loader;
			///Listener list
			std::list<MessageListener::Ptr> listeners;
			///Callback map for listeners
			CallbackMap callbacks;
		};
	}
}

/* SEATRAC_SEATRACCORE_H */
#endif
