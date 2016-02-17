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
#include <labust/seatrac/buddy_usbl.h>
#include <labust/comms/caddy/surface_handler.h>
#include <labust/comms/caddy/diver_handler.h>
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/seatrac_definitions.h>
#include <labust/seatrac/mediator.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/packer.h>

#include <pluginlib/class_list_macros.h>

#include <string>

using namespace labust::seatrac;
using namespace labust::comms::caddy;

BuddyUSBL::BuddyUSBL():
		pinger(sender, registrations),
		ping_rate(0)
{
	registrations[DatReceive::CID].push_back(Mediator<DatReceive>::makeCallback(
			boost::bind(&BuddyUSBL::onData,this,_1)));
}

BuddyUSBL::~BuddyUSBL()
{
	run_flag = false;
	worker.join();
}

bool BuddyUSBL::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	ph.param("ping_rate", ping_rate, ping_rate);

	nav_sub = nh.subscribe("position", 1, &BuddyUSBL::onEstimatedPos, this);

	handlers[SURFACE_ID].reset(new SurfaceHandler());
	handlers[DIVER_ID].reset(new DiverHandler());
	handlers[DIVER_ID]->configure(nh,ph);
	handlers[SURFACE_ID]->configure(nh,ph);

	run_flag = true;
	worker = boost::thread(boost::bind(&BuddyUSBL::run, this));

	return true;
}

void BuddyUSBL::onEstimatedPos(const auv_msgs::NavSts::ConstPtr& msg)
{
	boost::mutex::scoped_lock lock(message_mux);
	///TODO add substraction from init point and init point broadcast
  message.offset_x = msg->position.north;
  message.offset_y = msg->position.east;
  message.course = msg->orientation.yaw*180/M_PI;
  message.speed = msg->gbody_velocity.x;
  message.depth = msg->position.depth;
  message.altitude = msg->altitude;

  //TODO Handler battery_info, mission status and leak_info

  message.diver_offset_x = diver.position.north;
  message.diver_offset_y = diver.position.east;
}

void BuddyUSBL::run()
{
	ros::Rate rate((ping_rate==0)?1:ping_rate);

	bool ping_diver=true;

	while(ros::ok() && run_flag)
	{
		//Create the report
		DatSendCmd::Ptr data(new DatSendCmd());
		data->dest = ping_diver?DIVER_ID:SURFACE_ID;
		data->msg_type = AMsgType::MSG_REQU;

		boost::mutex::scoped_lock lock(message_mux);
		SeatracMessage::DataBuffer buf;
		labust::tools::encodePackable(message,&buf);
		data->data.assign(buf.begin(),buf.end());
		lock.unlock();

		ROS_INFO("Pinging: %d", data->dest);
		if (!pinger.send(boost::dynamic_pointer_cast<SeatracMessage>(data), TIMEOUT))
		{
			ROS_ERROR("BuddyUSBL: Message sending failed.");
		}

		//Reset the update if no reply was received
		//if (pinger.isError()){	}

		if (ping_rate) rate.sleep();
		ping_diver = !ping_diver;
	}
}

void BuddyUSBL::onData(const labust::seatrac::DatReceive& msg)
{
	HandlerMap::iterator it=handlers.find(msg.acofix.src);
	if (it != handlers.end())
	{
		(*handlers[msg.acofix.src])(msg);
	}
	else
	{
		ROS_WARN("No acoustic data handler found in BuddyUSBL for ID=%d.",msg.acofix.src);
	}
}

PLUGINLIB_EXPORT_CLASS(labust::seatrac::BuddyUSBL, labust::seatrac::DeviceController)
