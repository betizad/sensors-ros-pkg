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
 *
 *  Author : Dula Nad
 *  Created: 23.01.2013.
 *********************************************************************/
#include <labust/seatrac/seatrac_sim.h>
#include <labust/seatrac/seatrac_factory.h>
#include <pluginlib/class_list_macros.h>

using namespace labust::seatrac;

SeatracSim::SeatracSim(){}

bool SeatracSim::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	navsts = nh.subscribe<auv_msgs::NavSts>("navsts", 1,
			&SeatracSim::onNavSts, this);
	medium_in = nh.subscribe<underwater_msgs::MediumTransmission>("medium_in", 1,
			&SeatracSim::onMediumTransmission, this);

	medium_out = nh.advertise<underwater_msgs::MediumTransmission>("medium_out",1);

	return true;
}

void SeatracSim::onNavSts(const auv_msgs::NavSts::ConstPtr& msg)
{
	boost::shared_ptr<StatusResp> resp(new StatusResp());

	//Setup flags
	resp->status.status_output.ACC_CAL = 0;
	resp->status.status_output.AHRS_COMP_DATA = 0;
	resp->status.status_output.AHRS_RAW_DATA = 0;
	resp->status.status_output.ATTITUDE = 1;
	resp->status.status_output.ENVIRONMENT = 0;
	resp->status.status_output.MAG_CAL = 0;

	resp->status.attitude[Status::ROLL] = msg->orientation.roll*Status::ATT_SC;
	resp->status.attitude[Status::PITCH] = msg->orientation.pitch*Status::ATT_SC;
	resp->status.attitude[Status::YAW] = msg->orientation.yaw*Status::ATT_SC;

	boost::mutex::scoped_lock l(callback_mux);
	if (callback) callback(boost::dynamic_pointer_cast<SeatracMessage>(resp));
}

bool SeatracSim::send(const SeatracMessage::ConstPtr& msg)
{

	return true;
}

bool SeatracSim::resend()
{
	return true;
}

void SeatracSim::onMediumTransmission(const
				underwater_msgs::MediumTransmission::ConstPtr& msg)
{

}


PLUGINLIB_EXPORT_CLASS(labust::seatrac::SeatracSim, labust::seatrac::SeatracComms)

