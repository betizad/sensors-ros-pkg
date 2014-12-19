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

	medium_in = nh.subscribe<underwater_msgs::MediumTransmission>("medium_out", 3,
			&AcMediumSim::onMediumTransmission, this);

	medium_out = nh.advertise<underwater_msgs::MediumTransmission>("medium_in",16);
}

void AcMediumSim::onNavSts(int id, const auv_msgs::NavSts::ConstPtr& msg)
{
	//No need to protect with mutex with a single spinner
	nodes[id] = *msg;
}

bool AcMediumSim::onRegistration(underwater_msgs::AcSimRegister::Request& request,
		underwater_msgs::AcSimRegister::Response& response)
{
	if (request.navsts_topic.empty())
	{
		ROS_ERROR("Empty topic name for node ID=%d", request.node_id);
		return false;
	}

	if (nodes.find(request.node_id) != nodes.end())
	{
		ros::NodeHandle nh;

		ROS_INFO("Registering node ID=%d", request.node_id);
		//Add subscription to node and bind with node_id
		subs[request.node_id] = boost::bind(&AcMediumSim::onNavSts, this,
				request.node_id, _1);
		nodes[request.node_id] = auv_msgs::NavSts();

		/*boost::mutex::scoped_lock ltt(transport_mux);
		transport_timers[request.node_id] = nh.createTimer(ros::Duration(0.1),
				boost::bind(&AcMediumSim::receiveMessage, this,
						request.node_id,
						underwater_msgs::MediumTransmission::Ptr(), _1),
						false, false);
		ltt.unlock();*/

	/*	boost::mutex::scoped_lock ldt(delivery_mux);
		delivery_timers[request.node_id] = nh.createTimer(ros::Duration(0.1),
				boost::bind(&AcMediumSim::transportMessage, this,
						request.node_id,
						underwater_msgs::MediumTransmission::Ptr(), _1),
						false, false);*/
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
	NavStsMap::const_iterator it(nodes.find(msg->sender));
	if (it == nodes.end())
	{
		ROS_ERROR("Trying to publish from unregistered node.");
		return;
	}

	underwater_msgs::MediumTransmission msgout(*msg);
	msgout.position = it->second;

	DistanceMap dist;
	this->getDistances(msg->sender, it->second, dist);

	this->distributeToMedium(dist, msgout);

	//TODO Determine at what time the data can be transmitted to each node
	//TODO Throw dice for success and start transmission thread
}

void AcMediumSim::distributeToMedium(const DistanceMap& dist,
		const underwater_msgs::MediumTransmission& msg)
{
	boost::mutex::scoped_lock l(transport_mux);
	ros::NodeHandle nh;

	for (DistanceMap::const_iterator it = dist.begin();
			it != dist.end();
			++it)
	{
		//TODO Determine if node can hear the transmission
		//Get timer list
		std::list<ros::Timer>& timer_list(transport_timers.find(it->first)->second);
		std::list<ros::Timer>::iterator lit = processTimerList(timer_list);
		if (lit == timer_list.end())
		{
			timer_list.insert(nh.createTimer(ros::Duration(0.1),
					boost::bind(&AcMediumSim::transportMessage, this,
							it->first,
							underwater_msgs::MediumTransmission::Ptr(), _1),
							false, false));
		}
		//Clean dead timers
		for (std::list<ros::Timer>::const_iterator it2 = timer_list.begin();
				it2 != timer_list.end();
				++it2
				)
		{
			if (!it2->hasPending())
			{

			}
		}
		tit->second.push_back(nh.createTimer())
	}
}

void AcMediumSim::processTimerList(std::list<ros::Timer>& list)
{
  //Clean dead timers
	bool foundDead(false);
	std::list<ros::Timer>::const_iterator it = list.begin();
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

	//Create new if no dead iterator found
	if (!foundDead) retit = list.end();

	return retit;
}

void AcMediumSim::transportMessage(int node_id,
		underwater_msgs::MediumTransmission::ConstPtr msg,
		const ros::TimerEvent& event)
{
	//Message arrived to the node_id transponder
	/*boost::mutex::scoped_lock l(timers_mux);

	TimerMap::const_iterator it(delivery_timers.find(node_id));
	if (it != delivery_timers.end())
	{
		//Is already receiving a different message
		if (it->second.hasPending())
		{
			it->second.stop();
		}
		else
		{
			it->second.setPeriod(ros::Duration())
		}
	}
	*/
}

void AcMediumSim::receiveMessage(int node_id,
		underwater_msgs::MediumTransmission::ConstPtr msg,
		const ros::TimerEvent& event)
{
	boost::mutex::scoped_lock l(medium_mux);
	medium_out.publish(msg);
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

