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
 *  Created: 26.03.2013.
 *********************************************************************/
#include <auv_msgs/NavSts.h>
#include <ros/ros.h>

#include <vector>

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"static_beacon");
	ros::NodeHandle nh,ph("~");

	ros::Publisher pos = nh.advertise<auv_msgs::NavSts>("position", 1);

	std::vector<double> position(3,0), orientation(3,0);
	ph.param("position", position, position);
	ph.param("orientation", orientation, orientation);

	enum {x=0,y,z};
	enum {roll=0,pitch,yaw};
	auv_msgs::NavSts out;
	out.position.north = position[x];
	out.position.east = position[y];
	out.position.depth = position[z];
	out.orientation.roll = orientation[roll];
	out.orientation.pitch = orientation[pitch];
	out.orientation.yaw = orientation[yaw];

  out.header.stamp = ros::Time::now();
  out.header.frame_id = "local";

	//Latch the position of the device.
	ros::Rate rate(10);
	while (ros::ok())
	{
		pos.publish(out);
		rate.sleep();
	}
}
