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
#include <labust/comms/uros/uros_messages.h>


#include <misc_msgs/RhodamineAdc.h>
#include <auv_msgs/NavSts.h>
#include <std_msgs/Bool.h>
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
			enum {LLBITS_OUT=10, LLBITS_IN=10};
		public:
			///Main constructor
			UROSModemController();
			///Default destructor
			~UROSModemController();

			///Listener configuration.
			bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph);

		private:
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
				latlon.setInitLatLon(position.origin.latitude,
							position.origin.longitude);
			}

			///Handle the average flag report
			void onAvg(const std_msgs::Bool::ConstPtr& msg)
			{
				this->avg = msg->data;
			}
			/**
			 * Handles incoming acoustic data.
			 */
			void onData(const labust::seatrac::DatReceive& data);

			///Handle ping requests.
			void onPing(const labust::seatrac::PingReq& data);

			///Stop the vehicle on timeout
			void onTimeout(const ros::TimerEvent& e);

			///The command flag publisher.
			ros::Publisher cmd_pub;
			///Navigation update publisher.
			ros::Publisher nav_pub;
			///The ADC information subscription.
			ros::Subscriber adc_sub;
			///The navigation state subscription.
			ros::Subscriber state_sub;
			///Rhodamine average flag
			ros::Subscriber rhodamine_avg;

			///The last known position
			auv_msgs::NavSts position;
			///Position mutex
			boost::mutex position_mux;
			///The lat-lon encoder and tracker.
			labust::tools::Bits2LatLon latlon;

			///Add some comms timer
			ros::Timer nocomms;
			///The communication safety timeout
			double comms_timeout;
			///Safety mutex for the timer
			boost::mutex timer_mux;
			///Average detected
			bool avg;
			///Ping only flag
			bool empty_reply;

			labust::comms::uros::RhodamineData max_rhodamine;
		};
	}
}

/* USBL_COMMS_UROS_MODEM_CONTROLLER_H */
#endif
