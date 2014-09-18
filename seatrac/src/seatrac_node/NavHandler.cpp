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
#include <labust/seatrac/NavHandler.hpp>
#include <labust/seatrac/SeatracCID.hpp>
#include <labust/seatrac/SeatracMessages.hpp>
#include <labust/preprocessor/clean_serializator.hpp>

#include <underwater_msgs/USBLFix.h>
#include <geometry_msgs/PointStamped.h>

#include <boost/archive/binary_iarchive.hpp>

#include <iosfwd>

PP_LABUST_CLEAN_ARRAY_ISERIALIZATOR_IMPL(boost::archive::binary_iarchive)


using namespace labust::seatrac;

NavHandler::NavHandler()
{
	this->onInit();
}

void NavHandler::onInit()
{
	ros::NodeHandle nh, ph("~");

	usblFix = nh.advertise<underwater_msgs::USBLFix>("usbl_fix",1);
	//For backward compatibility
	relativePos = nh.advertise<geometry_msgs::PointStamped>("usbl_nav",1);
}

void NavHandler::operator()(int type, std::vector<uint8_t>& payload)
{
	//Just for safety
	if (type != CID_XCVR::fix) return;

	std::istringstream in;
	in.rdbuf()->pubsetbuf(reinterpret_cast<char*>(payload.data()), payload.size());
	boost::archive::binary_iarchive inSer(in, boost::archive::no_header);
	XcvrFix fix;
	inSer >> fix;
	ROS_INFO("Fix:");
	ROS_INFO("\t Beacon ID:%d", fix.beaconId);
	ROS_INFO("\t Signal valid:%d", fix.signal_valid);
	ROS_INFO("\t Depth valid:%d", fix.depth_valid);
	ROS_INFO("\t Range valid:%d", fix.range_valid);
	ROS_INFO("\t Range:%d", fix.range_dist);
	ROS_INFO("\t Remote depth:%d", fix.depth_remote);
	ROS_INFO("\t East:%d, North:%d, Depth:%d", fix.position[0], fix.position[1], fix.position[2]);

	if (fix.depth_valid)	ROS_WARN("Depth aided positioning not implemented.");

	//Is anything valid ?
	bool isValid = fix.range_valid || fix.signal_valid || fix.position_valid;

	if (isValid)
	{
		underwater_msgs::USBLFix::Ptr outfix(new underwater_msgs::USBLFix());
		outfix->beacon = fix.beaconId;
		outfix->position.x = fix.position[north]/1000.;
		outfix->position.y = fix.position[east]/1000.;
		outfix->position.z = fix.position[depth]/1000.;

		outfix->range = fix.range_dist/1000.;
		outfix->sound_speed = fix.vos/10.;
		outfix->elevation = fix.signal_elevation;
		outfix->bearing = fix.signal_azimuth;

		outfix->type = underwater_msgs::USBLFix::FULL_FIX;
		if (fix.range_valid && !fix.signal_valid)
		{
			outfix->type = underwater_msgs::USBLFix::RANGE_ONLY;
		}
		else if (fix.signal_valid && !fix.range_valid)
		{
			outfix->type = underwater_msgs::USBLFix::AZIMUTH_ONLY;
		}

		outfix->header.frame_id = "usbl_frame";
		outfix->header.stamp = ros::Time::now();
		usblFix.publish(outfix);

		//Backward compatible
		geometry_msgs::PointStamped::Ptr outpoint(new geometry_msgs::PointStamped());
		outpoint->point = outfix->position;
		outpoint->header.frame_id = "usbl_frame";
		outpoint->header.stamp = outfix->header.stamp;
		relativePos.publish(outpoint);
	}


}

