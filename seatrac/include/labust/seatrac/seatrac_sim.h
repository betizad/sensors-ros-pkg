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
#ifndef SEATRAC_SEATRACSIM_H
#define SEATRAC_SEATRACSIM_H
#include <labust/seatrac/seatrac_comms.h>

#include <auv_msgs/NavSts.h>
#include <underwater_msgs/MediumTransmission.h>
#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

namespace labust
{
	namespace seatrac
	{
		/**
		 * The class implements the Seatrac ROS simulation.
		 */
		class SeatracSim : public virtual SeatracComms
		{
		public:
			/**
			 * Main constructor
			 */
			SeatracSim();
			/**
			 * Configures the serial port based on the ROS node handle.
			 */
			bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph);

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
			///Navsts handler
			void onNavSts(const auv_msgs::NavSts::ConstPtr& msg);
			///Medium transmission handler
			void onMediumTransmission(const
				underwater_msgs::MediumTransmission::ConstPtr& msg);

			///Device position subscriber
			ros::Subscriber navsts;
			///Incoming data from medium subscriber
			ros::Subscriber medium_in;
			///Outgoing data to medium
			ros::Publisher medium_out;

			///The message callback
			CallbackType callback;
			///Muxer for the callback
			boost::mutex callback_mux;
		};
	}
}

/* SEATRAC_SEATRACSIM_H */
#endif
