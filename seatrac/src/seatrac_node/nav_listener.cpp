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
#include <labust/seatrac/nav_listener.h>
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/mediator.h>
#include <labust/tools/GeoUtilities.hpp>
#include <pluginlib/class_list_macros.h>

#include <underwater_msgs/USBLFix.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <string>
#include <Eigen/Dense>

using namespace labust::seatrac;

NavListener::NavListener():
	listener(buffer),
	use_ahrs(false),
	ahrs_delay(0),
	vos(0.0)
{
    registrations[PingReq::CID].push_back(Mediator<PingReq>::makeCallback(
                    boost::bind(&NavListener::onAcoFixMessage<PingReq>,this,_1)));
	registrations[PingResp::CID].push_back(Mediator<PingResp>::makeCallback(
					boost::bind(&NavListener::onAcoFixMessage<PingResp>,this,_1)));
	registrations[DatReceive::CID].push_back(Mediator<DatReceive>::makeCallback(
					boost::bind(&NavListener::onAcoFixMessage<DatReceive>,this,_1)));
	registrations[StatusResp::CID].push_back(Mediator<StatusResp>::makeCallback(
					boost::bind(&NavListener::onStatus,this,_1)));
}

bool NavListener::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	//Configure transponder information
	ph.param("use_ahrs", use_ahrs, use_ahrs);
	std::vector<int> tx;
	ph.param("transponders", tx, tx);
	std::vector<std::string> txnames;
	ph.param("transponder_names", txnames, txnames);
	//Set default values for missing transponder names
	for(int i=0; i<tx.size(); ++i)
	{
		//Add default name
		if (i>=txnames.size())
		{
			std::stringstream out;
			out<<"transponder_";
			out<<tx[i];
			txnames.push_back(out.str());
		}

		//Initialize publisher
		navsts_pub[tx[i]] = nh.advertise<auv_msgs::NavSts>(txnames[i]+"/usbl_navsts",1);
		fix_pub[tx[i]] = nh.advertise<underwater_msgs::USBLFix>(txnames[i]+"/usbl_fix",1);
		point_pub[tx[i]] = nh.advertise<geometry_msgs::Vector3Stamped>(
						txnames[i]+"/position_usbl",1);
	}

	return true;
}

void NavListener::processAcoFix(const AcoFix& fix)
{
	//Give some debug output
	ROS_DEBUG("Fix:");
	ROS_DEBUG("\t Destination:%d", fix.dest);
	ROS_DEBUG("\t Beacon ID:%d", fix.src);
	ROS_DEBUG("\t Signal valid:%d", fix.flags.USBL_VALID);
	ROS_DEBUG("\t Depth valid:%d", fix.flags.POSITION_ENHANCED);
	ROS_DEBUG("\t Range valid:%d", fix.flags.RANGE_VALID);
	ROS_DEBUG("\t Range:%d", fix.range.count);
	ROS_DEBUG("\t Count rssi:%d", int(fix.usbl.rssi.size()));
	ROS_DEBUG("\t East:%d, North:%d, Depth:%d", fix.position[AcoFix::y],
					fix.position[AcoFix::x], fix.position[AcoFix::z]);

	PublisherMap::iterator it;
	if ((it = point_pub.find(fix.src)) == point_pub.end())
	{
		ROS_WARN("No publisher for beacon id=%d", fix.src);
		return;
	}

	ros::Time cur_time(ros::Time::now());
	if (fix.flags.POSITION_ENHANCED) ROS_DEBUG("\t Enhanced position for "
					"transponder: %d", fix.src);

	underwater_msgs::USBLFix::Ptr fix_out(new underwater_msgs::USBLFix());
	//Add range information
	if (fix.flags.RANGE_VALID)
	{
		fix_out->range = float(fix.range.dist)/AcoFix::RANGE_SC;
		fix_out->type = underwater_msgs::USBLFix::RANGE_ONLY;
	}
	//Add angle information
	if (fix.flags.USBL_VALID)
	{
		fix_out->bearing = float(fix.usbl.azimuth)/AcoFix::ANGLE_SC;
		fix_out->elevation = float(fix.usbl.elevation)/AcoFix::ANGLE_SC;
		//If range is already valid the position will be valid as well
		//otherwise the azimuth is only valid
		fix_out->type = underwater_msgs::USBLFix::AZIMUTH_ONLY;
	}
	//Add position information
	if (fix.flags.POSITION_VALID)
	{
		//Get the relative position
		Eigen::Vector3d fixpos(
						float(fix.position[AcoFix::x])/AcoFix::RANGE_SC,
						float(fix.position[AcoFix::y])/AcoFix::RANGE_SC,
						float(fix.position[AcoFix::z])/AcoFix::RANGE_SC);

		try
		{
			geometry_msgs::TransformStamped transformDeg;
			transformDeg = buffer.lookupTransform("local", "usbl_frame", ros::Time(ahrs_delay));
			//Correct the relative measurement with the local AHRS if needed
			if (use_ahrs)
			{
				Eigen::Quaternion<double> quat(transformDeg.transform.rotation.w,
								transformDeg.transform.rotation.x,
								transformDeg.transform.rotation.y,
								transformDeg.transform.rotation.z);
				fixpos = quat.matrix()*fixpos;
			}

			this->calculateNavSts(fix_out->position, fixpos, transformDeg);
		}
		catch(tf2::TransformException& ex)
		{
			ROS_WARN("%s",ex.what());
		}

		geometry_msgs::Vector3Stamped::Ptr pos(new geometry_msgs::Vector3Stamped());
		pos->vector.x = float(fix.position[AcoFix::x])/AcoFix::RANGE_SC;
		pos->vector.y = float(fix.position[AcoFix::y])/AcoFix::RANGE_SC;
		pos->vector.z = float(fix.position[AcoFix::z])/AcoFix::RANGE_SC;
		pos->header.stamp = cur_time;
		it->second.publish(pos);

		fix_out->relative_position.x = pos->vector.x;
		fix_out->relative_position.y = pos->vector.y;
		fix_out->relative_position.z = pos->vector.z;
		fix_out->type = underwater_msgs::USBLFix::FULL_FIX;
	}
	//If anything was valid publish the fix data
	if (fix.flags.USBL_VALID || fix.flags.RANGE_VALID || fix.flags.POSITION_VALID)
	{
		fix_out->header.stamp = cur_time;
		fix_out->position.header.stamp = cur_time;
		fix_out->sound_speed = fix.vos;
		fix_pub[fix.src].publish(fix_out);
		navsts_pub[fix.src].publish(fix_out->position);
	}
}

void NavListener::calculateNavSts(auv_msgs::NavSts& nav, const Eigen::Vector3d& pos,
				const geometry_msgs::TransformStamped& trans)
{
	nav.header.frame_id = "local";
	nav.position.north = trans.transform.translation.x + pos(0);
	nav.position.east = trans.transform.translation.y + pos(1);
	nav.position.depth = trans.transform.translation.z + pos(2);
	
	try
	{
		geometry_msgs::TransformStamped transformDeg;
		transformDeg = buffer.lookupTransform("worldLatLon", "local", ros::Time(0));

		std::pair<double, double> diffAngle = labust::tools::meter2deg(nav.position.north, 
					nav.position.east,
					//The latitude angle
					transformDeg.transform.translation.y);
		nav.origin.latitude = transformDeg.transform.translation.y;
		nav.origin.longitude = transformDeg.transform.translation.x;
		nav.global_position.latitude = transformDeg.transform.translation.y + diffAngle.first;
		nav.global_position.longitude = transformDeg.transform.translation.x + diffAngle.second;
	}
	catch(tf2::TransformException& ex)
	{
		ROS_WARN("%s",ex.what());
	}
}

void NavListener::onStatus(const StatusResp& resp)
{
	if (resp.status.status_output.ENVIRONMENT)
	{
		vos = float(resp.status.env.vos)/Status::VOS_SC;
	}
}

PLUGINLIB_EXPORT_CLASS(labust::seatrac::NavListener, labust::seatrac::MessageListener)
