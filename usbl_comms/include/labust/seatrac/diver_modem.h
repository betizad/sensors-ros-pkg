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
#ifndef USBL_COMMS_DIVER_MODEM_H
#define USBL_COMMS_DIVER_MODEM_H
#include <labust/seatrac/device_controller.h>
#include <labust/seatrac/seatrac_messages.h>
#include <labust/comms/caddy/caddy_messages.h>

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
		class DiverModem : virtual public DeviceController
		{
			enum {TIMEOUT=4, BUDDY_ID=3, DIVER_ID=2, SURFACE_ID=1};
		public:
			///Main constructor
			DiverModem();
			///Default destructor
			~DiverModem();

			///Listener configuration.
			bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph);

		private:
			///Handles the diver position.
			void onNavSts(const auv_msgs::NavSts::ConstPtr& msg);
			///Handles incoming acoustic data.
			void onData(const labust::seatrac::DatReceive& data);
			///Helper methods
			int adaptmeas(double value, int a, int b, double q);
			double decodemeas(double value, int a, int b, double q);

			///The surface navigation publisher
			ros::Publisher surfacenav_pub;
			///The buddy navigation publisher
			ros::Publisher buddynav_pub;
			///The diver navigation publisher
			ros::Publisher divernav_pub;
			///The navigation state subscription.
			ros::Subscriber state_sub;
		};
	}
}

/* USBL_COMMS_UROS_MODEM_CONTROLLER_H */
#endif
