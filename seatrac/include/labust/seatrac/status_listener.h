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
#ifndef SEATRAC_STATUSLISTENER_H
#define SEATRAC_STATUSLISTENER_H
#include <labust/seatrac/message_listener.h>
#include <labust/seatrac/seatrac_messages.h>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <ros/ros.h>

#include <vector>
#include <string>

namespace labust
{
	namespace seatrac
	{
		/**
		 * The class implements the status publisher and decoder.
		 */
		class StatusListener : virtual public MessageListener
		{
		public:
			///Main constructor
			StatusListener();

			///Listener configuration.
			bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph);
			///Register for messages
			const RegisterMap& getRegistrations(){return registrations;}

		private:
			///Handle status message
			void onStatus(const StatusResp& resp);

			///Attitude publisher
			ros::Publisher attitude;
			///Temperature publisher
			ros::Publisher temperature;
			///Diagnostics publisher
			ros::Publisher diagnostic;
			///The cumulative diagnostic status
			diagnostic_msgs::DiagnosticStatus status;
			///Calibration fit flag
			bool calib_flag;
			///Voltage supply flag
			bool supply_flag;

			///Minimum voltage
			double min_voltage;

			///Helper function for attitude processing
			void processAttitude(const vec3si& attitude);
			///Helper function for environment processing
			void processEnvironment(const EnvStatus& env);
			///Helper function for magnetic calibration processing
			void processMagneticCalibration(const MagCalibration& mag);

			///Helper function for adding data to diagnostics
			void addToDiagnostics(const std::vector<std::string> names, const std::vector<double> values);
		};
	}
}

/* SEATRAC_STATUSLISTENER_H */
#endif
