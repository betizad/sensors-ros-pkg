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
#include <labust/comms/uros/uros_messages.h>
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

	nav_sub = nh.subscribe("navsts", 1, &BuddyUSBL::onEstimatedPos, this);

	divernav_pub = nh.advertise<auv_msgs::NavSts>("diver_nav", 1);
	surfacenav_pub = nh.advertise<auv_msgs::NavSts>("surface_nav",	1);

	run_flag = true;
	worker = boost::thread(boost::bind(&BuddyUSBL::run, this));

	return true;
}

int BuddyUSBL::adaptmeas(double value, int a, int b, double q)
{
	assert(q !=0 && "Quantization of zero is not realistic.");
	value = labust::math::coerce(value, a*q, b*q);
	return int(value/q - a);
}

double BuddyUSBL::decodemeas(double value, int a, int b, double q)
{
	assert(q !=0 && "Quantization of zero is not realistic.");
	return q*(value+a);
}

void BuddyUSBL::onEstimatedPos(const auv_msgs::NavSts::ConstPtr& msg)
{
	boost::mutex::scoped_lock lock(message_mux);
	///TODO: add scaling and rounding to be
	/// auto-generated from a message definition
	/// add signed, unsigned flag and quantization
	/// value OR min, max and quantization
	/// ADD STORAGE TYPE SO THAT ENCODING IS DONE IN PLACE
	message.msg_id = BuddyReport::MSG_ID;
	enum {POS_A = -512, POS_B=512};
	const double POS_QUANT = 0.1;

	///TODO add substraction from init point and init point broadcast
  message.offset_x = adaptmeas(msg->position.north, POS_A, POS_B, POS_QUANT);
  message.offset_y = adaptmeas(msg->position.east, POS_A, POS_B, POS_QUANT);

  ///TODO add heading if stationary or course if moving
  /// depending on speed value
  double heading(msg->orientation.yaw*180/M_PI), u(msg->gbody_velocity.x),
  		v(msg->gbody_velocity.y), speed(sqrt(u*u+v*v));

  if (speed > 0.1) heading = 180*atan2(u,v)/M_PI;
  if (heading < 0) heading = heading + 360;
  message.course = adaptmeas(heading, 0, 1024, 360.0/1024.0);
  message.speed = adaptmeas(speed, 0, 16, 1.0/16.0);
  message.depth = adaptmeas(msg->position.depth, 0, 128, 0.5);

  message.diver_offset_x = adaptmeas(diver.position.north, POS_A, POS_B, POS_QUANT);
  message.diver_offset_y = adaptmeas(diver.position.east, POS_A, POS_B, POS_QUANT);
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
	if (msg.acofix.src == DIVER_ID)
	{
		///TODO: add handling of messages based on message ID
		///TODO: create acofix processor similar to pinger class
		/// in order to process position in-place and fuse with
		/// payload information

		DiverNav diver;
		if (!labust::tools::decodePackable(msg.data, &diver))
		{
			ROS_WARN("BuddyUSBL: Empty message received from modem.");
			return;
		}

		auv_msgs::NavSts::Ptr divernav(new auv_msgs::NavSts());
		divernav->orientation.yaw = labust::math::wrapRad(
				M_PI*decodemeas(diver.heading, 0, 1024, 360.0/1024.0)/180);
		divernav->position.depth = decodemeas(diver.depth, 0, 128, 0.5);
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

PLUGINLIB_EXPORT_CLASS(labust::seatrac::BuddyUSBL, labust::seatrac::DeviceController)
