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
#include <labust/seatrac/data_listener.h>
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/mediator.h>
#include <pluginlib/class_list_macros.h>

#include <underwater_msgs/ModemTransmission.h>
#include <string>
#include <Eigen/Dense>

using namespace labust::seatrac;

DataListener::DataListener()
{
	registrations[DatReceive::CID].push_back(Mediator<DatReceive>::makeCallback(
					boost::bind(&DataListener::onData,this,_1)));
}

bool DataListener::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	//Configure transponder information
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
		data_pub[tx[i]] = nh.advertise<underwater_msgs::ModemTransmission>(txnames[i]+"/usbl_data",1);
	}

	return true;
}

void DataListener::onData(const DatReceive& resp)
{
	if (resp.data.size())
	{
		underwater_msgs::ModemTransmission::Ptr datout(new underwater_msgs::ModemTransmission());
		datout->header.stamp = ros::Time::now();
		datout->action = underwater_msgs::ModemTransmission::RECEIVED;
		datout->sender = resp.acofix.src;
		datout->receiver = resp.acofix.dest;
		datout->payload.assign(resp.data.begin(), resp.data.end());
		PublisherMap::iterator it = data_pub.find(datout->sender);
		if (it!=data_pub.end())
		{
			data_pub[datout->sender].publish(datout);
		}
		else
		{
			ROS_WARN("Publisher for payload data of node %d not found.",datout->sender);
		}
	}
	else
	{
		ROS_WARN("Empty data message received.");
	}
}

PLUGINLIB_EXPORT_CLASS(labust::seatrac::DataListener, labust::seatrac::MessageListener)
