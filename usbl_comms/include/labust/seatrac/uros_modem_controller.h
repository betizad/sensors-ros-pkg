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
#ifndef USBL_COMMS_UROS_MODEM_CONTROLLER_H
#define USBL_COMMS_UROS_MODEM_CONTROLLER_H
#include <labust/seatrac/device_controller.h>
#include <labust/seatrac/seatrac_messages.h>
#include <labust/tools/latlon_encoder.h>

#include <misc_msgs/RhodamineAdc.h>
#include <auv_msgs/NavSts.h>
#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

namespace labust
{
	namespace seatrac
	{
		/**
		 * The class implements the status publisher and decoder.
		 */
		class UROSModemController : virtual public DeviceController
		{
			enum {llbits=14};
		public:
			///Main constructor
			UROSModemController();
			///Default destructor
			~UROSModemController();

			///Listener configuration.
			bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph);
			///Register for messages
			const RegisterMap& getRegistrations(){return registrations;}
			///Register callback for interrogation.
			void registerCallback(const SeatracComms::CallbackType& callback){this->sender = callback;};

		private:
			///Registration map
			RegisterMap registrations;
			///Sender callback
			SeatracComms::CallbackType sender;

			/**
			 * Handles outgoing messages requests.
			 */
			void onAdc(const misc_msgs::RhodamineAdc::ConstPtr& msg);
			/**
			 * Handles the local position.
			 */
			void onNavSts(const auv_msgs::NavSts::ConstPtr& msg)
			{
				boost::mutex::scoped_lock l(position_mux);
				position = *msg;
			}
			/**
			 * Handles incoming acoustic data.
			 */
			void onData(const labust::seatrac::DatReceive& data);

			///The command flag publisher.
			ros::Publisher cmd_pub;
			///Navigation update publisher.
			ros::Publisher nav_pub;
			///The ADC information subscription.
			ros::Subscriber adc_sub;
			///The navigation state subscription.
			ros::Subscriber state_sub;

			///The last known position
			auv_msgs::NavSts position;
			///Position mutex
			boost::mutex position_mux;
			///The lat-lon encoder and tracker.
			labust::tools::Bits2LatLon latlon;
		};
	}
}

/* USBL_COMMS_UROS_MODEM_CONTROLLER_H */
#endif
