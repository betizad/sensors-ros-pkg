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
#ifndef USBL_COMMS_BUDDY_USBL_H
#define USBL_COMMS_BUDDY_USBL_H
#include <labust/seatrac/device_controller.h>
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/pinger.h>
#include <labust/comms/caddy/caddy_messages.h>

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
		class BuddyUSBL : virtual public DeviceController
		{
			enum {TIMEOUT=4, DIVER_ID=2, SURFACE_ID=1};
		public:
			///Main constructor
			BuddyUSBL();
			///Default destructor
			~BuddyUSBL();

			///Listener configuration.
			bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph);

		private:
			///The USBL pinger
			Pinger pinger;

			///Handle the Buddy position estimated position position.
			void onEstimatedPos(const auv_msgs::NavSts::ConstPtr& msg);
			///Handle the Diver position estimated position position.
			void onDiverPos(const auv_msgs::NavSts::ConstPtr& msg)
			{
				diver = *msg;
			}
			///Handles incoming acoustic data.
			void onData(const labust::seatrac::DatReceive& data);
			///The pinging function
			void run();
			///Helper methods
			int adaptmeas(double value, int a, int b, double q);
			double decodemeas(double value, int a, int b, double q);

			///Position estimate subscriber.
			ros::Subscriber nav_sub;
			///The diver navigation information publisher.
			ros::Subscriber diverpos_sub;
			///The diver navigation information publisher.
			ros::Publisher divernav_pub;
			///The surface navigation state subscription.
			ros::Publisher surfacenav_pub;

			///The next message mux.
			boost::mutex message_mux;
			///The ping rate
			double ping_rate;
			///The payload message
			labust::comms::caddy::BuddyReport message;
			///The diver position
			auv_msgs::NavSts diver;

			///The worker thread
			boost::thread worker;
			///The run flag for the worker thread.
			bool run_flag;
		};
	}
}

/* USBL_COMMS_BUDDY_USBL_H */
#endif
