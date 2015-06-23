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
#ifndef USBL_COMMS_UROS_USBL_CONTROLLER_H
#define USBL_COMMS_UROS_USBL_CONTROLLER_H
#include <labust/seatrac/device_controller.h>
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/pinger.h>
#include <labust/comms/uros/uros_messages.h>
#include <labust/tools/latlon_encoder.h>

#include <auv_msgs/NavSts.h>
#include <std_msgs/UInt8.h>
#include <ros/ros.h>

#include <boost/thread.hpp>

namespace labust
{
	namespace seatrac
	{
		/**
		 * The class implements the status publisher and decoder.
		 */
		class UROSUSBLController : virtual public DeviceController
		{
			enum {LLBITS_OUT=18, LLBITS_IN=18, TIMEOUT=4};
		public:
			///Main constructor
			UROSUSBLController();
			///Default destructor
			~UROSUSBLController();

			///Listener configuration.
			bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph);

		private:
			///The USBL pinger
			Pinger pinger;

			/**
			 * Handle the abort message.
			 */
			void onCommand(const std_msgs::UInt8::ConstPtr& msg)
			{
				boost::mutex::scoped_lock l(message_mux);
				message.cmd_flag = msg->data;
				msg_updated = true;
			}

			/**
			 * Handles the estimated position position.
			 */
			void onEstimatedPos(const auv_msgs::NavSts::ConstPtr& msg)
			{
				boost::mutex::scoped_lock l(message_mux);
				labust::tools::LatLon2Bits ll;
				latlon.setInitLatLon(msg->origin.latitude,
						msg->origin.longitude);
				ll.convert(msg->global_position.latitude,
						msg->global_position.longitude, LLBITS_OUT);
				message.lat = ll.lat;
				message.lon = ll.lon;
				//msg_updated = true;
			}
			/**
			 * Handles incoming acoustic data.
			 */
			void onData(const labust::seatrac::DatReceive& data);

			///The pinging function
			void run();

			///The command flag publisher.
			ros::Subscriber cmd_sub;
			///Position estimate subscriber.
			ros::Subscriber nav_sub;
			///The ADC information publisher.
			ros::Publisher adc_pub;
			///The navigation state subscription.
			ros::Publisher state_pub;
			///The current status publisher.
			ros::Publisher status_pub;

			///The next message to send.
			labust::comms::uros::LupisUpdate message;
			///The next message mux.
			boost::mutex message_mux;
			///The lat-lon encoder and tracker.
			labust::tools::Bits2LatLon latlon;
			///The Lupis transponder ID.
			int id;
			///The message flag for indicating an update.
			bool msg_updated;
			///The ping rate
			double ping_rate;

			///The worker thread
			boost::thread worker;
			///The run flag for the worker thread.
			bool run_flag;
		};
	}
}

/* USBL_COMMS_UROS_USBL_CONTROLLER_H */
#endif
