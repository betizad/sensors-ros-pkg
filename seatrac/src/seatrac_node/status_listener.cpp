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
#include <labust/seatrac/status_listener.h>
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/mediator.h>
#include <pluginlib/class_list_macros.h>

#include <auv_msgs/RPY.h>
#include <sensor_msgs/Temperature.h>

#include <string>


using namespace labust::seatrac;

StatusListener::StatusListener():
		calib_flag(true),
		supply_flag(true),
		min_voltage(12.0)
{
	registrations[StatusResp::CID] = Mediator<StatusResp>::makeCallback(
			boost::bind(&StatusListener::onStatus,this,_1));
}

bool StatusListener::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	//Configure desired outputs and rate
	attitude = nh.advertise<auv_msgs::RPY>("usbl_attitude",1);
	temperature = nh.advertise<sensor_msgs::Temperature>("temperature",1);
	diagnostic = nh.advertise<diagnostic_msgs::DiagnosticStatus>("diagnostic",1);

	return true;
}

void StatusListener::processAttitude(const vec3si& attitude)
{
	auv_msgs::RPY::Ptr rpy(new auv_msgs::RPY());
	rpy->roll = float(attitude[Status::YAW])/Status::ATT_SC;
	rpy->pitch = float(attitude[Status::PITCH])/Status::ATT_SC;
	rpy->yaw = float(attitude[Status::YAW])/Status::ATT_SC;
	this->attitude.publish(rpy);
}

void StatusListener::processEnvironment(const EnvStatus& env)
{
	const int elsize = 5;
	std::vector<std::string> names = {"supply","pressure","depth","temperature","sound_speed"};
	std::vector<double> values = {float(env.supply)/Status::VOLTAGE_SC,
			float(env.pressure)/Status::PRESSURE_SC,
			float(env.depth)/Status::DEPTH_SC,
			float(env.temp)/Status::TEMP_SC,
			float(env.vos)/Status::VOS_SC
	};

	this->addToDiagnostics(names, values);

	sensor_msgs::Temperature::Ptr temp(new sensor_msgs::Temperature());
	temp->temperature = float(env.temp)/Status::TEMP_SC;
	temp->header.stamp = ros::Time::now();
	temperature.publish(temp);

	//Set flags
	supply_flag = (float(env.supply)/Status::VOLTAGE_SC) < min_voltage;
}


void StatusListener::processMagneticCalibration(const MagCalibration& mag)
{
	const int elsize = 2;
	std::vector<std::string> names = {"calibration_fit","calibration_valid"};
	std::vector<double> values = {double(mag.fit), double(mag.valid != 0)};

	this->addToDiagnostics(names, values);

	//Set flags
	calib_flag = mag.valid;
}

void StatusListener::addToDiagnostics(const std::vector<std::string> names,
		const std::vector<double> values)
{
	if (values.size() != names.size())
	{
		ROS_WARN("StatusListener: Key-value sizes for diagnostic vector do not match.");
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

void StatusListener::onStatus(const StatusResp& resp)
{
	//Read sequentially - the order of functions matters !
	if (resp.status.status_output.ENVIRONMENT)
		processEnvironment(resp.status.env);
	if (resp.status.status_output.ATTITUDE)
		processAttitude(resp.status.attitude);
	if (resp.status.status_output.MAG_CAL)
		processMagneticCalibration(resp.status.mag_cal);
	if (resp.status.status_output.ACC_CAL){};
	if (resp.status.status_output.AHRS_RAW_DATA){};
	if (resp.status.status_output.AHRS_COMP_DATA){};

	//Send diagnostic messages
	this->status.hardware_id = "SeaTrac";
	this->status.name = "SeaTrac";

	if (calib_flag || supply_flag)
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

PLUGINLIB_EXPORT_CLASS(labust::seatrac::StatusListener, labust::seatrac::MessageListener)
