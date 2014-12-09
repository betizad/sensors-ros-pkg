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
#include <labust/seatrac/seatrac_core.h>
#include <ros/ros.h>

using namespace labust::seatrac;

SeatracCore::SeatracCore():
			comms_loader("seatrac",
					"labust::seatrac::SeatracComms"),
			listener_loader("seatrac",
					"labust::seatrac::MessageListener"),
			control_loader("seatrac",
					"labust::seatrac::DeviceController")

{
	this->onInit();
}

SeatracCore::~SeatracCore(){}

void SeatracCore::onInit()
{
	ros::NodeHandle nh, ph("~");
	try
	{
		this->setupPlugins(nh, ph);
		//Connect to the communication layer
		comms->registerCallback(boost::bind(&SeatracCore::incomingMsg, this, _1));
	}
	catch (std::exception& e)
	{
		ROS_ERROR("SeatracCore::onInit(): Exception caught: %s", e.what());
	}


	/*
	std::string portName("/dev/ttyUSB0");
	int baud(115200);
	ph.param("PortName",portName,portName);
	ph.param("Baud",baud,baud);

	ros::Rate r(1);
	bool setupOk(false);
	while (!(setupOk = comms.connect(portName, baud)) && ros::ok())
	{
		ROS_ERROR("SeaTracNode::Failed to open port.");
		r.sleep();
	}

	if (setupOk)
	{
		ph.param("auto_mode",autoMode,autoMode);
		ph.param("master",isMaster,isMaster);
		///\todo Add automatic detection of capabilities
		ph.param("usbl",isUsbl,isUsbl);
		ph.param("timeout",ping_timeout,ping_timeout);
		ph.param("id", transponderId, transponderId);
		ph.param("only_ack", onlyAck, onlyAck);
		///\todo remove this hardcoded part and replace with configuration
		if (transponderId == 1) trackId.push_back(2);
		else trackId.push_back(1);

		//Register message handlers
		dispatch[CID_XCVR::fix] = boost::bind(&NavHandler::operator(), &nav,_1,_2);
		dispatch[CID_STATUS::status] = boost::bind(&StatusHandler::operator(), &stats,_1,_2);

		dataSub = nh.subscribe<underwater_msgs::ModemTransmission>("outgoing_data",
				0, &SeaTracNode::onOutgoing,this);
		dataSubBW = nh.subscribe<std_msgs::String>("outgoing_data_bw",
						0, &SeaTracNode::onOutgoingBW,this);
		opMode = nh.subscribe<std_msgs::Bool>("auto_mode",	0,
				&SeaTracNode::onAutoMode,this);
		dataPub = nh.advertise<underwater_msgs::ModemTransmission>("incoming_data",1);
		dataPubBW = nh.advertise<std_msgs::String>("incoming_data_bw",1);
		allMsg = nh.advertise<std_msgs::UInt8MultiArray>("all_msgs",1);
		usblTimeout = nh.advertise<std_msgs::Bool>("usbl_timeout",1);
		remoteDepth = nh.advertise<std_msgs::Float32>("remote_depth",1);

		this->start();

		//Register main callback
		comms.registerCallback(
				boost::bind(&SeaTracNode::incomingMsg, this, _1, _2));
	}
	 */
}

void SeatracCore::setupPlugins(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	ROS_INFO("loading plugins ...");

	//Configure comms
	std::string commsplug, conplug;
	ph.param("comms_plugin",commsplug,commsplug);
	comms = comms_loader.createInstance(commsplug);
	if ((comms == 0) || !comms->configure(nh, ph))
		throw std::runtime_error("SeatracCore: Comms configuration failed.");
	ROS_INFO("Comms plugin: '%s' loaded", commsplug.c_str());

	//Configure controller
	ph.param("controller_plugin", conplug, conplug);
	controller = control_loader.createInstance(conplug);
	if ((controller == 0) || !controller->configure(nh, ph))
		throw std::runtime_error("SeatracCore: Controller configuration failed.");
	controller->registerCallback(boost::bind(&SeatracCore::outgoingMsg, this, _1));
	ROS_INFO("Controller plugin: '%s' loaded", conplug.c_str());

	//Configure listeners
	std::vector<std::string> listener_list;
	ph.param("listener_plugins", listener_list, listener_list);

	for (int i=0; i<listener_list.size(); ++i)
	{
		try
		{
			MessageListener::Ptr ltemp = listener_loader.createInstance(listener_list[i]);
			//Configure
			if (!ltemp->configure(nh, ph))
			{
				ROS_WARN("Listener %s failed to configure and will be skipped.",
						listener_list[i].c_str());
				continue;
			}
			//Add listener
			listeners.push_back(ltemp);
			//Add callbacks
			const MessageListener::RegisterMap& lmap = ltemp->getRegistrations();
			for(MessageListener::RegisterMap::const_iterator it = lmap.begin();
					it != lmap.end();
					++it)
			{
				callbacks[it->first].push_back(it->second);
			}

			ROS_INFO("Listener plugin: '%s' loaded", listener_list[i].c_str());
		}
		catch (std::exception& e)
		{
			ROS_ERROR("Failed loading listener plugin:%s",e.what());
		}
	}

	ROS_INFO(" loaded.");
}


bool SeatracCore::incomingMsg(const SeatracMessage::ConstPtr& msg)
{
	CallbackMap::const_iterator it = callbacks.find(msg->getCid());

	if (it != callbacks.end())
	{
		ROS_DEBUG("Found subscription.");
		const CallbackList& tlist = it->second;
		for (CallbackList::const_iterator it = tlist.begin();
				it != tlist.end();
				++it
		)
		{
			ROS_DEBUG("Dispatch message.");
			(*it)(msg);
		}
	}
	else
	{
		ROS_WARN("Message ignored: %d", msg->getCid());
	}
	return true;
}

bool SeatracCore::outgoingMsg(const SeatracMessage::ConstPtr& msg)
{
	ROS_INFO("SeatracCore: Relay controller message.");
	return comms->send(msg);
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"seatrac_core");
	SeatracCore core;
	ros::spin();

	return 0;
}

