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
#ifndef SIMULATION_ACMEDIUMSIM_H
#define SIMULATION_ACMEDIUMSIM_H

#include <auv_msgs/NavSts.h>
#include <underwater_msgs/MediumTransmission.h>
#include <underwater_msgs/AcSimRegister.h>
#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

#include <map>

namespace labust
{
	namespace seatrac
	{
		/**
		 * The class implements the acoustic medium simulator ROS wrapper.
		 */
		class AcMediumSim
		{
			///The node navigation state map
			typedef std::map<int, auv_msgs::NavSts> NavStsMap;
			typedef std::map<int, ros::Timer> TimerMap;
			typedef std::map<int, std::list<ros::Timer> > TimerVectorMap;
			typedef boost::function<void(const auv_msgs::NavSts::ConstPtr&)> Callback;
			typedef std::map<int, Callback> SubscriberMap;
			typedef std::map<int, double> DistanceMap;

		public:
			///Main constructor
			AcMediumSim();
			///Default destructor
			~AcMediumSim();

			///Configures the medium.
			void onInit();

		private:
			///Navsts handler
			void onNavSts(int id, const auv_msgs::NavSts::ConstPtr& msg);
			///Medium transmission handler
			void onMediumTransmission(const
				underwater_msgs::MediumTransmission::ConstPtr& msg);
			///Service call handler
			bool onRegistration(underwater_msgs::AcSimRegister::Request& request,
					underwater_msgs::AcSimRegister::Response& response);
			///Helper method for distance calculations
			void getDistances(int node_id, const auv_msgs::NavSts& npos, DistanceMap& map);
			///Helper function to start medium delay
			void distributeToMedium(const DistanceMap& dist,
					const underwater_msgs::MediumTransmission& msg);

			/**
			 * Helper method to simulate data transmission through medium. This
			 * method is called when the first bit of the message arrives at the
			 * receiver. In case the receiver is already in process of receiving
			 * this method will cancel that the old and newely arrived receptions
			 * to simulate conflict.
			 */
			void transportMessage(int node_id,
					underwater_msgs::MediumTransmission::ConstPtr msg,
					const ros::TimerEvent& event);
			/**
			 * Helper method to simulate arrived data at the other end.
			 * This method is called if during the whole message sending
			 * nothing interrupted the transmission.
			 */
			void receiveMessage(int node_id,
					underwater_msgs::MediumTransmission::ConstPtr msg,
					const ros::TimerEvent& event);

			///Helper function for timer list processing
			void processTimerList(std::list<ros::Timer>& list);

			///Medium input subscriber
			ros::Subscriber medium_in;
			///Medium output publisher
			ros::Publisher medium_out;
			///Service for node registration
			ros::ServiceServer registration;
			///Registration map
			NavStsMap nodes;
			///Map of NavSts subscribers for each node.
			SubscriberMap subs;
			///Map of transport timers
			TimerVectorMap transport_timers;
			///Map of delivery timers
			TimerMap delivery_timers;
			///Muxer for transport timers
			boost::mutex transport_mux;
			///Muxer for delivery timers
			boost::mutex delivery_mux;
			///Muxer for the medium publisher
			boost::mutex medium_mux;
			///Velocity of sound (default: 1475)
			double vos;
			///Flag for occupied medium (default: false)
			bool mediumOccupied;
		};
	}
}

/* SIMULATION_ACMEDIUMSIM_H */
#endif
