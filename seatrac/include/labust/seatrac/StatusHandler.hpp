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
#ifndef STATUSHANDLER_HPP_
#define STATUSHANDLER_HPP_
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <ros/ros.h>

#include <boost/archive/binary_iarchive.hpp>

#include <map>

namespace labust
{
	namespace seatrac
	{
		/**
		 * The class implements the status handler publisher and decoder.
		 * \todo Extract message decoding from payload to a templated function
		 * \todo Add verbose diagnostics and basic diagnostic options
		 * \todo Add missing handlers
		 */
		class StatusHandler
		{
			enum {yaw=0, pitch, roll};
		public:
			/**
			 * Main constructor
			 */
			StatusHandler();
			/**
			 * Initialize and setup controller.
			 */
			void onInit();

			/**
			 * Main handling operator.
			 */
			void operator()(int type, std::vector<uint8_t>& payload);

		private:
			///Attitude publisher
			ros::Publisher attitude;
			///Diagnostics publisher
			ros::Publisher diagnostic;
			///The cumulative diagnostic status
			diagnostic_msgs::DiagnosticStatus status;
			///Calibration fit flag
			bool calibFlag;
			///Voltage supply flag
			bool supplyFlag;

			///Minimum voltage
			double minVoltage;

			///Helper function for attitude processing
			void processAttitude(boost::archive::binary_iarchive& inSer);
			///Helper function for environment processing
			void processEnvironment(boost::archive::binary_iarchive& inSer);
			///Helper function for magnetic calibration processing
			void processMagneticCalibration(boost::archive::binary_iarchive& inSer);

			///Helper function for adding data to diagnostics
			void addToDiagnostics(const std::vector<std::string> names, const std::vector<double> values);
		};
	}
}

/* STATUSHANDLER_HPP_ */
#endif
