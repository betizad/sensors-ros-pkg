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
#include <labust/seatrac/StatusHandler.hpp>
#include <labust/seatrac/SeatracCID.hpp>
#include <labust/seatrac/SeatracMessages.hpp>
#include <labust/preprocessor/clean_serializator.hpp>

#include <auv_msgs/RPY.h>
#include <sensor_msgs/Temperature.h>


#include <string>

PP_LABUST_CLEAN_ARRAY_ISERIALIZATOR_IMPL(boost::archive::binary_iarchive)

using namespace labust::seatrac;

StatusHandler::StatusHandler():
				calibFlag(true),
				supplyFlag(true),
				minVoltage(12.0)
{
	this->onInit();
}

void StatusHandler::onInit()
{
	ros::NodeHandle nh;
	//Configure desired outputs and rate
	attitude = nh.advertise<auv_msgs::RPY>("usbl_attitude",1);
	temperature = nh.advertise<sensor_msgs::Temperature>("temperature",1);
	diagnostic = nh.advertise<diagnostic_msgs::DiagnosticStatus>("diagnostic",1);
}

void StatusHandler::processAttitude(boost::archive::binary_iarchive& inSer)
{
	vec3si attitude;
	inSer >> attitude;

	auv_msgs::RPY::Ptr rpy(new auv_msgs::RPY());
	rpy->roll = attitude[roll]/10.;
	rpy->pitch = attitude[pitch]/10.;
	rpy->yaw = attitude[yaw]/10.;
	this->attitude.publish(rpy);
}

void StatusHandler::processEnvironment(boost::archive::binary_iarchive& inSer)
{
	EnvStatus env;
	inSer >> env;

	const int elsize = 5;
	std::vector<std::string> names = {"supply","pressure","depth","temperature","sound_speed"};
	std::vector<double> values = {env.supply/1000., env.pressure/1000., env.depth/10.,
			env.temp/10., env.vos/10.
	};

	this->addToDiagnostics(names, values);

	sensor_msgs::Temperature::Ptr temp(new sensor_msgs::Temperature());
	temp->temperature = env.temp/10.;
	temperature.publish(temp);

	//Set flags
	supplyFlag = (env.supply/1000.) < minVoltage;
}

void StatusHandler::processMagneticCalibration(boost::archive::binary_iarchive& inSer)
{
	MagCalibrationStatus mag;
	inSer >> mag;

	const int elsize = 2;
	std::vector<std::string> names = {"calibration_fit","calibration_valid"};
	std::vector<double> values = {double(mag.cal_fit), double(mag.cal_valid != 0)};

	this->addToDiagnostics(names, values);

	//Set flags
	calibFlag = mag.cal_valid;
}

void StatusHandler::addToDiagnostics(const std::vector<std::string> names,
		const std::vector<double> values)
{
	if (values.size() != names.size())
	{
		ROS_WARN("StatusHandler: Key-value sizes for diagnostic vector do not match.");
	}

	int min = names.size();
	if (values.size() < min) min = values.size();

	for (int i=0; i<min; ++i)
	{
		diagnostic_msgs::KeyValue val;
		val.key = names[i];
		std::stringstream out;
		out << values[i];
		val.value = out.str();
		status.values.push_back(val);
	}
}

void StatusHandler::operator ()(int type, std::vector<uint8_t>& payload)
{
	if (type != CID_STATUS::status) return;
	///\todo Extract this in
	std::istringstream in;
	in.rdbuf()->pubsetbuf(reinterpret_cast<char*>(payload.data()), payload.size());
	boost::archive::binary_iarchive inSer(in, boost::archive::no_header);
	StatusHeader status;
	inSer >> status;

	//Read sequentially - the order of functions matters !
	if (status.status_bits.ENVIRONMENT) processEnvironment(inSer);
	if (status.status_bits.ATTITUDE) processAttitude(inSer);
	if (status.status_bits.MAG_CAL) processMagneticCalibration(inSer);

	if (status.status_bits.ACC_CAL)
	{
		AccCalibrationStatus acc;
		inSer >> acc;
		//
	}

	if (status.status_bits.AHRS_RAW_DATA)
	{
		AHRSData raw;
		inSer >> raw;
		//
	}


	if (status.status_bits.AHRS_COMP_DATA)
	{
		AHRSData comp;
		inSer >> comp;
		//
	}

	//Send diagnostic messages
	this->status.hardware_id = "SeaTrac";
	this->status.name = "SeaTrac";

	if (calibFlag || supplyFlag)
	{
		this->status.level = diagnostic_msgs::DiagnosticStatus::WARN;
	}
	else
	{
		this->status.level = diagnostic_msgs::DiagnosticStatus::OK;
	}
	this->diagnostic.publish(this->status);
	//Clean diagnostics
	this->status.values.clear();
}

