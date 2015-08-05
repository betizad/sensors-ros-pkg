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
#include <labust/simulation/ac_medium_sim.h>
#include <ros/ros.h>

using namespace labust::seatrac;

AcMediumSim::AcMediumSim():
				vos(1475),
				mediumOccupied(false)
{
	this->onInit();
}

AcMediumSim::~AcMediumSim(){}

void AcMediumSim::onInit()
{
	ros::NodeHandle nh, ph("~");

	medium_in = nh.subscribe("medium_out", 3, &AcMediumSim::onMediumTransmission, this);

	medium_out = nh.advertise<underwater_msgs::MediumTransmission>("medium_in",16);

	registration = nh.advertiseService("register_modem",&AcMediumSim::onRegistration, this);
}

void AcMediumSim::onNavSts(int id, const auv_msgs::NavSts::ConstPtr& msg)
{
	//No need to protect with mutex with a single spinner
	boost::mutex::scoped_lock l(state_mux);
	nodes[id] = *msg;
}

bool AcMediumSim::onRegistration(underwater_msgs::AcSimRegister::Request& request,
		underwater_msgs::AcSimRegister::Response& response)
{
	boost::mutex::scoped_lock l(state_mux);
	if (request.navsts_topic.empty())
	{
		ROS_ERROR("Empty topic name for node ID=%d", request.node_id);
		return false;
	}

	if (nodes.find(request.node_id) == nodes.end())
	{
		ros::NodeHandle nh;

		ROS_INFO("Registering node ID=%d", request.node_id);
		//Add subscription to node and bind with node_id
		subs[request.node_id] = nh.subscribe<auv_msgs::NavSts>(request.navsts_topic, 1,
				boost::bind(&AcMediumSim::onNavSts, this, request.node_id, _1));
		nodes[request.node_id] = auv_msgs::NavSts();
	}
	else
	{
		ROS_WARN("Node with ID=%d is already registered.", request.node_id);
		return false;
	}

	return true;
}

void AcMediumSim::onMediumTransmission(const
	underwater_msgs::MediumTransmission::ConstPtr& msg)
{
	//The node has to be registered (if no NavSts message is received the default is NavSts()).
	boost::mutex::scoped_lock l(state_mux);
	NavStsMap::const_iterator it(nodes.find(msg->sender));
	if (it == nodes.end())
	{
		ROS_ERROR("Trying to publish from unregistered node.");
		return;
	}
	l.unlock();

	DistanceMap dist;
	this->getDistances(msg->sender, it->second, dist);
	this->distributeToMedium(dist, msg);

	//TODO Determine at what time the data can be transmitted to each node
	//TODO Throw dice for success and start transmission thread
}

void AcMediumSim::distributeToMedium(const DistanceMap& dist,
		const underwater_msgs::MediumTransmission::ConstPtr msg)
{
	boost::mutex::scoped_lock l(transport_mux);
	ros::NodeHandle nh;

	for (DistanceMap::const_iterator it = dist.begin();
			it != dist.end();
			++it)
	{
		//TODO Determine if node can hear the transmission
		//Get timer list
		std::list<ros::Timer>& timer_list(transport_timers[it->first]);
		processTimerList(timer_list);
		timer_list.push_back(nh.createTimer(ros::Duration(it->second/vos),
					boost::bind(&AcMediumSim::transportMessage, this, it->first, msg, _1),
							true, true));
		ROS_DEBUG("Sent message from %d to %d. Distance is %f.",msg->sender, it->first, it->second);
	}
}

void AcMediumSim::processTimerList(std::list<ros::Timer>& list)
{
	//Clean dead timers
	if (list.empty()) return;

	std::list<ros::Timer>::iterator it = list.begin();
	while (it != list.end())
	{
		if (!it->hasPending())
		{
			it = list.erase(it);
		}
		else
		{
			++it;
		}
	}
}

void AcMediumSim::transportMessage(int node_id,
		underwater_msgs::MediumTransmission::ConstPtr msg,
		const ros::TimerEvent& event)
{
	//Message arrived to the node_id transponder
	ROS_DEBUG("Message arrived at %d.", node_id);
	boost::mutex::scoped_lock l(delivery_mux);

	TimerMap::iterator it(delivery_timers.find(node_id));
	if (it != delivery_timers.end())
	{
		//Is already receiving a different message
		if (it->second.hasPending())
		{
			ROS_WARN("Conflict in communication on acoustic node %d.", node_id);
			it->second.stop();
			return;
		}
	}

	ros::NodeHandle nh;
	delivery_timers[node_id] = nh.createTimer(ros::Duration(msg->duration),
				boost::bind(&AcMediumSim::receiveMessage, this, node_id,
						msg,_1),true,true);
}

void AcMediumSim::receiveMessage(int node_id,
		underwater_msgs::MediumTransmission::ConstPtr msg,
		const ros::TimerEvent& event)
{
	ROS_INFO("Message (sender=%d, receiver=%d) arrived at node %d.",
					msg->sender, msg->receiver, node_id);
	underwater_msgs::MediumTransmission::Ptr msgout(new underwater_msgs::MediumTransmission(*msg));
	msgout->listener_id = node_id;
	///TODO Check mutexing here
	boost::mutex::scoped_lock ls(state_mux);
	NavStsMap::const_iterator it(nodes.find(msg->sender));
	msgout->position = it->second;
	ls.unlock();
	boost::mutex::scoped_lock l(medium_mux);
	medium_out.publish(msgout);
}

void AcMediumSim::getDistances(int node_id, const auv_msgs::NavSts& npos, DistanceMap& map)
{
	for (NavStsMap::const_iterator it=nodes.begin();
			it!=nodes.end();
			++it)
	{
		if (it->first == node_id) continue;

		double dx = it->second.position.north - npos.position.north;
		double dy = it->second.position.east - npos.position.east;
		double dz = it->second.position.depth - npos.position.depth;
		map[it->first] = sqrt(dx*dx + dy*dy + dz*dz);
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"ac_medium");
	AcMediumSim core;
	ros::spin();

	return 0;
}

