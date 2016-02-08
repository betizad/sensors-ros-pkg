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
#include <labust/seatrac/diver_modem.h>
#include <labust/comms/caddy/caddy_messages.h>
#include <labust/comms/caddy/buddy_handler.h>
#include <labust/comms/caddy/surface_handler.h>
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

DiverModem::DiverModem()
{
	registrations[DatReceive::CID].push_back(Mediator<DatReceive>::makeCallback(
			boost::bind(&DiverModem::onData,this,_1)));
}

DiverModem::~DiverModem(){}

bool DiverModem::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	handlers[BUDDY_ID].reset(new BuddyHandler());
  handlers[SURFACE_ID].reset(new SurfaceHandler());
	handlers[BUDDY_ID]->configure(nh,ph);
	handlers[SURFACE_ID]->configure(nh,ph);

	state_sub = nh.subscribe("position",	1, &DiverModem::onNavSts, this);

	return true;
}

void DiverModem::onNavSts(const auv_msgs::NavSts::ConstPtr& msg)
{
	DatQueueClearCmd::Ptr clr(new DatQueueClearCmd());
	DatQueueSetCmd::Ptr cmd(new DatQueueSetCmd());
	cmd->dest = labust::seatrac::BEACON_ALL;
	std::vector<char> binary;
	DiverReport diver;
  diver.heading = 180*msg->orientation.yaw/M_PI;;
  diver.depth = msg->position.depth;
	labust::tools::encodePackable(diver, &binary);
	cmd->data.assign(binary.begin(),binary.end());

	if (!sender.empty())
	{
		sender(clr);
		sender(cmd);
	}
}

void DiverModem::onData(const labust::seatrac::DatReceive& msg)
{
	HandlerMap::iterator it=handlers.find(msg.acofix.src);
	if (it != handlers.end())
	{
		(*handlers[msg.acofix.src])(msg);
	}
	else
	{
		ROS_WARN("No acoustic data handler found in DiverModem for ID=%d.",msg.acofix.src);
	}
}

PLUGINLIB_EXPORT_CLASS(labust::seatrac::DiverModem, labust::seatrac::DeviceController)
