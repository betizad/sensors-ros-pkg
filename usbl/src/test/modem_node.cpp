/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, LABUST, UNIZG-FER
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
 *  Author: Dula Nad
 *  Created: 14.02.2013.
 *
 *  Modified by: Filip Mandic
 *  Modified on: 02.09.2014.
 *********************************************************************/
#include <labust/tritech/MTDevice.hpp>
#include <labust/tritech/mtMessages.hpp>
#include <labust/tritech/USBLMessages.hpp>
#include <labust/tritech/DiverMsg_adv.hpp>
#include <labust/tritech/mmcMessages.hpp>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <ros/ros.h>

#include <boost/bind.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

using namespace labust::tritech;

class AcousticModem{

public:

	AcousticModem(std::string port, int baud, int timeout, int mode):modem(port,baud), activeMode(mode),
																		max_seconds(timeout), modemBusy(false), timeout(timeout){

		switch(activeMode){

			case commOnly:
				setupCommOnly();
				break;

			case rangeOnly:
				setupRangeOnly();
				loop();
				break;

			default:
				break;
		}
	}

	~AcousticModem(){}

	void setupCommOnly(){

		ros::NodeHandle nh;
		subMsg = nh.subscribe<std_msgs::String>("modem_in", 1, &AcousticModem::onTransmit, this);
		pubMsg = nh.advertise<std_msgs::String>("modem_out",1);

		map[MTMsg::mtMiniModemData] = boost::bind(&AcousticModem::onReceive, this, _1);

		modem.registerHandlers(map);
	}

	void setupRangeOnly(){

		ros::NodeHandle nh;
		pubMsg = nh.advertise<std_msgs::Float32>("rangeMeas",1);
		pubTime = nh.advertise<std_msgs::UInt32>("timeMeas",1);

		map[MTMsg::mtMiniModemData] = boost::bind(&AcousticModem::onReceive, this, _1);

		modem.registerHandlers(map);
	}

	void reboot(){

		MTMsgPtr tmsg(new MTMsg());
		tmsg->txNode = 255;
		tmsg->rxNode = labust::tritech::Nodes::SlaveModem;
		tmsg->node = labust::tritech::Nodes::SlaveModem;
		tmsg->msgType = MTMsg::mtReboot;
		modem.send(tmsg);
		//Wait for modem to init
		ros::Duration(0.5).sleep();
	}

	void onReceive(labust::tritech::MTMsgPtr tmsg){

		boost::archive::binary_iarchive dataSer(*tmsg->data, boost::archive::no_header);

		if(activeMode == commOnly){

		} else if(activeMode == rangeOnly){

			//boost::mutex::scoped_lock lock(pingLock);

			MMCMsgShort data;
			dataSer>>data;
			uint32_t time;
			dataSer>>time;

			ROS_ERROR("%d",data.msgType);

			double soundspeed = 1475;
			double zeroDelay = 61015;
			double range(0);

			if (time > zeroDelay){
				range = 0.5 * (time - zeroDelay) * soundspeed * 1e-6;


				std::istream in(tmsg->data.get());
				std::cout<<"remaining:";
				while(!in.eof()){
						uint8_t c;
						in>>c;
						std::cout<<int(c)<<",";
				}
				std::cout<<std::endl;

				ROS_ERROR("Timing=%d => range=%f", time,range);

				std_msgs::Float32 rangeData;
				rangeData.data = range;
				pubMsg.publish(rangeData);

				std_msgs::UInt32 timeData;
				timeData.data = time;
				pubTime.publish(timeData);

			} else {

				ROS_ERROR("Zero measurement.");
			}

			lastMsg = ros::Time::now();
			modemBusy = false;
			modemCondition.notify_one();
		}
	}

	void onTransmit(const std_msgs::String::ConstPtr& data){

	}

	void getRange(){

		MTMsgPtr tmsg(new MTMsg());
		tmsg->txNode = 255;
		tmsg->rxNode = labust::tritech::Nodes::SlaveModem;
		tmsg->node = labust::tritech::Nodes::SlaveModem;
		tmsg->msgType = MTMsg::mtMiniModemCmd;

		MMCMsg mmsg;
		mmsg.msgType = labust::tritech::mmcGetRangeSync;
		mmsg.rxTmo = 2000;

		std_msgs::String::Ptr msg(new std_msgs::String);
		std::copy(msg->data.begin(), msg->data.end(), mmsg.data.begin());
		boost::archive::binary_oarchive ar(*tmsg->data, boost::archive::no_header);
		ar<<mmsg;
		modem.send(tmsg);

		ROS_ERROR("Send message.");

		/* Wait for response before continuing */
		modemBusy = true;
		boost::mutex::scoped_lock lock(pingLock);
		boost::system_time const timeout=boost::get_system_time()+boost::posix_time::seconds(timeout);
		while (modemBusy)
		{
			/* If there is no response from slave modem for timeout seconds reboot master modem??  */
			if (!modemCondition.timed_wait(lock,timeout)){

				ROS_ERROR("Rebooting modem");
				reboot();
				modemBusy = false;
				break;
			}
		}
	}

	void loop(){
		while(ros::ok()){

			getRange();
		}
	}

	/*****************************************************************
	 *** Class variables
	 ****************************************************************/

	enum {commOnly = 0, rangeOnly};

	ros::Subscriber subMsg;
	ros::Publisher pubMsg, pubTime;
	ros::Time lastMsg;


	MTDevice modem;
	MTDevice::HandlerMap map;

	int activeMode;
	int max_seconds;
	int timeout;

	/**
	 * The USBL status.
	 */
	bool modemBusy;
	/**
	 * The lock condition variable.
	 */
	boost::condition_variable modemCondition;

	/**
	 * The data and condition mux.
	 */
	boost::mutex dataMux, pingLock;



};

int main(int argc, char* argv[])
{

	ros::init(argc,argv,"modem_node");
	ros::NodeHandle nh, ph("~");

	std::string port("/dev/ttyUSB0");
	int baud(57600);
	int max_seconds(60);

	ph.param("PortName",port,port);
	ph.param("Baud",baud,baud);
	ph.param("timeout",max_seconds,max_seconds);

	AcousticModem AM(port,baud,max_seconds,AcousticModem::rangeOnly);

	ros::spin();

	return 0;
}



