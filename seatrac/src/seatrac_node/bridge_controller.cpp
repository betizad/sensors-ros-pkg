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
#include <labust/seatrac/bridge_controller.h>
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/seatrac_definitions.h>
#include <labust/seatrac/seatrac_factory.h>
#include <labust/seatrac/mediator.h>
#include <pluginlib/class_list_macros.h>

#include <string>
#include <sstream>

using namespace labust::seatrac;

BridgeController::BridgeController()
{
	registrations[ALL_MSG_CID] = boost::bind(&BridgeController::onMsg,this,_1);
}

bool BridgeController::onMsg(const SeatracMessage::ConstPtr& msg)
{
	std_msgs::String::Ptr out(new std_msgs::String());
	SeatracFactory::encodePacket(msg, &out->data);
	outgoing.publish(out);
	return true;
}

bool BridgeController::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	incoming = nh.subscribe<std_msgs::String>("incoming",
			1, &BridgeController::onIncoming,this);
	outgoing = nh.advertise<std_msgs::String>("outgoing", 1);
	return true;
}


void BridgeController::onIncoming(const std_msgs::String::ConstPtr& msg)
{
	SeatracMessage::Ptr in;
	if (SeatracFactory::decodePacket(&msg->data, in))
	{
		this->sender(in);
	}
	else
	{
		ROS_ERROR("BridgeController: packet decoding failed.");
	}
}

PLUGINLIB_EXPORT_CLASS(labust::seatrac::BridgeController, labust::seatrac::DeviceController)
