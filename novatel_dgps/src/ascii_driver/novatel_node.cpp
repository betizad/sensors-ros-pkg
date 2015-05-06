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
#include <labust/navigation/novatel_node.h>
#include <labust/navigation/novatel_messages.h>
#include <labust/archive/delimited_iarchive.hpp>
#include <labust/archive/delimited_oarchive.hpp>
#include <labust/preprocessor/clean_serializator.hpp>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>

#include <boost/bind.hpp>
#include <boost/serialization/string.hpp>

#include <sys/stat.h>

PP_LABUST_CLEAN_ARRAY_OSERIALIZATOR_IMPL(labust::archive::delimited_oarchive)
PP_LABUST_CLEAN_ARRAY_ISERIALIZATOR_IMPL(labust::archive::delimited_iarchive)

using namespace labust::navigation;

NovatelNode::NovatelNode()
{
	this->onInit();
}

NovatelNode::~NovatelNode(){}

void NovatelNode::onInit()
{
	ros::NodeHandle nh, ph("~");

	std::string port_name("/dev/ttyUSB0");
	int baud(115200);
	ph.param("port_name",port_name,port_name);
	ph.param("baud",baud,baud);

	//Try connecting
	while(ros::ok())
	{
		try
		{
			struct stat buffer;
			if (stat (port_name.c_str(), &buffer))
			{
				ROS_ERROR("Serial port does not exist yet.");
			}
			else
			{
				if (serial.connect(port_name, baud)) break;
			}
		}
		catch (std::exception& e)
		{
			ROS_ERROR("%s: Serial connection failed. %s",
					ros::this_node::getName().c_str(), e.what());
		}
		ros::Duration(1.0).sleep();
	}
	
	ROS_INFO("Opened serial port %s at baud %d.", port_name.c_str(), baud);

	fix_pub = nh.advertise<sensor_msgs::NavSatFix>("fix",1);
	vel_pub = nh.advertise<geometry_msgs::TwistStamped>("vel",1);

	serial.registerCallback(boost::bind(&NovatelNode::onData, this, _1));
}

bool NovatelNode::testHeader(const std::string& match, const std::string& stream)
{
	return stream.substr(0,match.size()) == match;
}

void NovatelNode::onBestPos(BestPos& pos)
{
	sensor_msgs::NavSatFix sat_fix;
	    sat_fix.header.frame_id = "/gps_frame";
	    enum {latitude=0, longitude=1, altitude=2};
	    sat_fix.latitude = pos.position[latitude];
	    sat_fix.longitude = pos.position[longitude];
	    sat_fix.altitude = pos.position[altitude];

	    if (pos.pos_type == "NONE")
	      sat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
	    else if ((pos.pos_type == "WAAS") ||
	             (pos.pos_type == "OMNISTAR") ||
	             (pos.pos_type == "OMNISTAR_HP") ||
	             (pos.pos_type == "OMNISTAR_XP") ||
	             (pos.pos_type == "CDGPS"))
	      sat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
	    else if ((pos.pos_type == "PSRDIFF") ||
	    				 (pos.pos_type == "L1_FLOAT") ||
	             (pos.pos_type == "NARROW_FLOAT") ||
	             (pos.pos_type == "WIDE_INT") ||
	             (pos.pos_type == "WIDE_INT") ||
	             (pos.pos_type == "NARROW_INT") ||
	             (pos.pos_type == "RTK_DIRECT_INS") ||
	             (pos.pos_type == "INS_PSRDIFF") ||
	             (pos.pos_type == "INS_RTKFLOAT") ||
	             (pos.pos_type == "INS_RTKFIXED"))
	      sat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
			else
			  sat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;


	    sat_fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
	    sat_fix.position_covariance[0] = std::pow(pos.position_stddev[latitude],2);
	    sat_fix.position_covariance[4] = std::pow(pos.position_stddev[longitude],2);
	    sat_fix.position_covariance[8] = std::pow(pos.position_stddev[altitude],2);

	    sat_fix.header.stamp = ros::Time::now();
	    fix_pub.publish(sat_fix);
}

void NovatelNode::onBestVel(BestVel& vel)
{
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    twist.header.frame_id = "local";
    twist.twist.linear.x = vel.hor_spd*cos(M_PI*vel.trk_gnd/180);
    twist.twist.linear.y = vel.hor_spd*sin(M_PI*vel.trk_gnd/180);
    twist.twist.linear.z = -vel.vert_spd;

    vel_pub.publish(twist);
}

void NovatelNode::onData(const std::string& data)
{
	ROS_INFO("Received message: %s", data.c_str());
	size_t p = data.find("#");
	if (p < data.size())
	{
		std::string packet = data.substr(p,data.size()-2);
		ROS_INFO("Received packet: %s", packet.c_str());
		//Replace ';' and '*' markers
		p = packet.find(";");
		packet[p] = ',';
		p = packet.find("*");
		packet[p] = ',';
		ROS_INFO("Updated packet: %s", packet.c_str());

		std::istringstream is;
		is.rdbuf()->pubsetbuf(&packet[0],packet.size());
		labust::archive::delimited_iarchive ia(is,',');

		//For more messages add a dispatcher
		if (testHeader("#BESTPOSA", packet))
		{
			ROS_INFO("BESTPOS message received.");
			BestPos pos;
			ia >> pos;
			this->onBestPos(pos);
		}
		else if (testHeader("#BESTVELA", packet))
		{
			ROS_INFO("BESTVEL message received.");
			BestVel vel;
			ia >> vel;
			this->onBestVel(vel);
		}
		else
		{
			ROS_WARN("Unknown message type.");
		}
	}


		/*if (test_header("$#NQ.RES", data))
		{
			NQRes dvl_data;
			int chk = labust::tools::getChecksum(reinterpret_cast<unsigned char*>(&data[15]), data.size()-3-15);
			std::istringstream is;
			is.rdbuf()->pubsetbuf(&data[0],size);
			labust::archive::delimited_iarchive ia(is);
			ia>>dvl_data;

			//if (error_code(dvl_data) == 0) publishDvlData(dvl_data);
			publishDvlData(dvl_data);
			ROS_INFO("Calculated checksum:calc=%d, recvd=%d", chk, dvl_data.checksum);

			//ROS_INFO("DVL decoded: header:%s, error:%s.",
			//		dvl_data.header.c_str(),
			//		dvl_data.error_code.c_str());
		}*/
}


int main(int argc, char* argv[])
{
	ros::init(argc,argv,"novatel_node");
	NovatelNode node;
	ros::spin();

	return 0;
}


