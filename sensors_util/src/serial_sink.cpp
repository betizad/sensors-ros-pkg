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
#include <labust/comms/ascii_serial.h>
#include <std_msgs/String.h>
#include <ros/ros.h>

void outgoing(const ros::Publisher& pub, const std::string& message)
{
	std_msgs::String::Ptr out(new std_msgs::String());
	out->data.assign(message);
	pub.publish(out);
}

void incoming(labust::comms::ASCIISerial& port, const std_msgs::String::ConstPtr& data)
{
	port.send(data->data + "\r\n");
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"serial_sink");

	ros::NodeHandle nh, ph("~");
	//Get serial port data
	std::string port_name("/dev/ttyUSB0");
	int baud(115200);
	ph.param("port_name",port_name,port_name);
	ph.param("baud",baud,baud);

	labust::comms::ASCIISerial serial;
	//Try connecting
	while(ros::ok())
	{
		try
		{
			if (serial.connect(port_name, baud)) break;
		}
		catch (std::exception& e)
		{
			ROS_ERROR("%s: Serial connection failed. %s",
							ros::this_node::getName().c_str(), e.what());
		}
		ros::Duration(1.0).sleep();
	}
	ROS_INFO("Opened serial port %s at baud %d.", port_name.c_str(), baud);

	ros::Publisher pub = nh.advertise<std_msgs::String>("incoming",1);
	ros::Subscriber sub = nh.subscribe<std_msgs::String>("outgoing",1,
					boost::bind(&incoming, boost::ref(serial),_1));

	serial.registerCallback(boost::bind(&outgoing, boost::ref(pub),_1));

	ros::spin();

	return 0;
}


