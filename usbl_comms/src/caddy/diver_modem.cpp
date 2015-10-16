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
	divernav_pub = nh.advertise<auv_msgs::NavSts>("diver_nav", 1);
	surfacenav_pub = nh.advertise<auv_msgs::NavSts>("surface_nav",	1);
	buddynav_pub = nh.advertise<auv_msgs::NavSts>("buddy_nav",	1);

	state_sub = nh.subscribe("position",	1, &DiverModem::onNavSts, this);

	return true;
}

int DiverModem::adaptmeas(double value, int a, int b, double q)
{
	assert(q !=0 && "Quantization of zero is not realistic.");
	value = labust::math::coerce(value, a*q, b*q);
	return int(value/q - a);
}

double DiverModem::decodemeas(double value, int a, int b, double q)
{
	assert(q !=0 && "Quantization of zero is not realistic.");
	return q*(value+a);
}

void DiverModem::onNavSts(const auv_msgs::NavSts::ConstPtr& msg)
{
	DatQueueClearCmd::Ptr clr(new DatQueueClearCmd());
	DatQueueSetCmd::Ptr cmd(new DatQueueSetCmd());
	cmd->dest = labust::seatrac::BEACON_ALL;
	std::vector<char> binary;
	DiverNav diver;
	double heading = 180*msg->orientation.yaw/M_PI;
  if (heading < 0) heading = heading + 360;
  diver.heading = adaptmeas(heading, 0, 1024, 360.0/1024.0);
  diver.depth = adaptmeas(msg->position.depth, 0, 128, 0.5);
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
	///TODO: Put decoders/encoders for each agent in a class to be shared
	/// between modem controllers

	if (msg.acofix.src == BUDDY_ID)
	{
		///TODO: add handling of messages based on message ID
		///TODO: create acofix processor similar to pinger class
		/// in order to process position in-place and fuse with
		/// payload information

		BuddyReport buddy;
		if (!labust::tools::decodePackable(msg.data, &buddy))
		{
			ROS_WARN("BuddyUSBL: Empty message received from modem.");
			return;
		}

		auv_msgs::NavSts::Ptr buddynav(new auv_msgs::NavSts());
		enum {POS_A = -512, POS_B=512};
		const double POS_QUANT = 0.1;

		///TODO add substraction from init point and init point broadcast
		buddynav->position.north = decodemeas(buddy.offset_x, POS_A, POS_B, POS_QUANT);
		buddynav->position.east = decodemeas(buddy.offset_y, POS_A, POS_B, POS_QUANT);
		buddynav->position.depth = decodemeas(buddy.depth, 0, 128, 0.5);
		buddynav->orientation.yaw = labust::math::wrapRad(
					M_PI*decodemeas(buddy.course, 0, 1024, 360.0/1024.0)/180);
	  buddynav->gbody_velocity.x = decodemeas(buddy.speed, 0, 16, 1.0/16.0);
		buddynav_pub.publish(buddynav);

		auv_msgs::NavSts::Ptr divernav(new auv_msgs::NavSts());
	  divernav->position.north = decodemeas(buddy.diver_offset_x, POS_A, POS_B, POS_QUANT);
	  divernav->position.east = decodemeas(buddy.diver_offset_y, POS_A, POS_B, POS_QUANT);
	  divernav_pub.publish(divernav);
	}
	else if (msg.acofix.src == SURFACE_ID)
	{
		SurfaceNav surf;
		if (!labust::tools::decodePackable(msg.data, &surf))
		{
			ROS_WARN("BuddyUSBL: Empty message received from modem.");
			return;
		}

		enum {POS_A = -512, POS_B=512};
		const double POS_QUANT = 0.1;

		auv_msgs::NavSts::Ptr surfnav(new auv_msgs::NavSts());
		surfnav->position.north = decodemeas(surf.offset_x, POS_A, POS_B, POS_QUANT);
		surfnav->position.east = decodemeas(surf.offset_y, POS_A, POS_B, POS_QUANT);
		surfnav->gbody_velocity.x = decodemeas(surf.speed, 0, 16, 1.0/16.0);
		surfnav->orientation.yaw = labust::math::wrapRad(
				M_PI*decodemeas(surf.course, 0, 1024, 360.0/1024.0)/180);
		surfacenav_pub.publish(surfnav);
	}

}

PLUGINLIB_EXPORT_CLASS(labust::seatrac::DiverModem, labust::seatrac::DeviceController)
