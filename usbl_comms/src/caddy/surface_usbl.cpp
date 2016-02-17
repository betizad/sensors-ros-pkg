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
#include <labust/seatrac/surface_usbl.h>
#include <labust/comms/caddy/caddy_messages.h>
#include <labust/comms/caddy/buddy_handler.h>
#include <labust/comms/caddy/diver_handler.h>
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/seatrac_definitions.h>
#include <labust/seatrac/mediator.h>
#include <labust/tools/packer.h>
#include <labust/math/NumberManipulation.hpp>

#include <pluginlib/class_list_macros.h>

#include <std_msgs/UInt8.h>
#include <sensor_msgs/NavSatFix.h>

#include <string>

using namespace labust::seatrac;
using namespace labust::comms::caddy;

SurfaceUSBL::SurfaceUSBL()
{
	registrations[DatReceive::CID].push_back(Mediator<DatReceive>::makeCallback(
			boost::bind(&SurfaceUSBL::onData,this,_1)));
}

SurfaceUSBL::~SurfaceUSBL(){}

bool SurfaceUSBL::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	//Configure defaults for the surface message
	//The command 0 represents NOP
	surf.mission_cmd = NOP;
	//Lawn mower parameter 0,0
	surf.lawn_length = 0;
	surf.lawn_width = 0;

	state_sub = nh.subscribe("position",	1, &SurfaceUSBL::onNavSts, this);
	surfacecmd_sub = nh.subscribe("surface_cmd",	1, &SurfaceUSBL::onMissionCmd, this);
	lawnmower_sub = nh.subscribe("lawnmower_req",	1, &SurfaceUSBL::onLawnMower, this);

	handlers[BUDDY_ID].reset(new BuddyHandler());
	handlers[DIVER_ID].reset(new DiverHandler());
	handlers[DIVER_ID]->configure(nh,ph);
	handlers[BUDDY_ID]->configure(nh,ph);

	return true;
}

void SurfaceUSBL::onNavSts(const auv_msgs::NavSts::ConstPtr& msg)
{
	static int cmdd = 0;
	DatQueueClearCmd::Ptr clr(new DatQueueClearCmd());
	DatQueueSetCmd::Ptr cmd(new DatQueueSetCmd());
	cmd->dest = labust::seatrac::BEACON_ALL;
	std::vector<char> binary;

	surf.offset_x = msg->position.north;
	surf.offset_y = msg->position.east;
	surf.course = msg->orientation.yaw*180/M_PI;
	surf.speed = msg->gbody_velocity.x;
	labust::tools::encodePackable(surf, &binary);
	cmd->data.assign(binary.begin(),binary.end());

	if (!sender.empty())
	{
		sender(clr);
		sender(cmd);
	}
}

void SurfaceUSBL::onMissionCmd(const std_msgs::UInt8::ConstPtr& msg)
{
	surf.mission_cmd = msg->data;
}

void SurfaceUSBL::onLawnMower(const caddy_msgs::LawnmowerReq::ConstPtr& msg)
{
	surf.lawn_width = msg->width;
	surf.lawn_length = msg->length;
	surf.mission_cmd = LAWN_MOWER;
}

void SurfaceUSBL::onData(const labust::seatrac::DatReceive& msg)
{
	HandlerMap::iterator it=handlers.find(msg.acofix.src);
	if (it != handlers.end())
	{
		(*handlers[msg.acofix.src])(msg);
	}
	else
	{
		ROS_WARN("No acoustic data handler found in SurfaceUSBL for ID=%d.",msg.acofix.src);
	}
}

PLUGINLIB_EXPORT_CLASS(labust::seatrac::SurfaceUSBL, labust::seatrac::DeviceController)
