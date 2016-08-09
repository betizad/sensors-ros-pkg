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
#ifndef SEATRAC_NAVLISTENER_H
#define SEATRAC_NAVLISTENER_H
#include <labust/seatrac/message_listener.h>
#include <labust/seatrac/seatrac_messages.h>

#include <auv_msgs/NavSts.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <Eigen/Dense>

#include <vector>
#include <string>
#include <map>

namespace labust
{
	namespace seatrac
	{
		/**
		 * The class implements the status publisher and decoder.
		 */
		class NavListener : virtual public MessageListener
		{
			typedef std::map<int, ros::Publisher> PublisherMap;
		public:
			///Main constructor
			NavListener();

			///Listener configuration.
			bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph);

		private:
			///Handle any response with an acofix message.
			template<class Type>
			void onAcoFixMessage(const Type& resp)
			{
				this->processAcoFix(resp.acofix);
			}

			///AcoFix message processor
			void processAcoFix(const AcoFix& fix);
			///Status processor
			void onStatus(const StatusResp& resp);
			///Calculate the full navigation solution
			void calculateNavSts(auv_msgs::NavSts& nav,	const Eigen::Vector3d& pos,
							const geometry_msgs::TransformStamped& trans);

			///Transponder fix publishers
			PublisherMap fix_pub;
			///Transponder fix publishers
			PublisherMap navsts_pub;
			///Transponder relative position publishers
			PublisherMap point_pub;

			///Internal status data
			double vos;

			///Internal AHRS compensation
			bool use_ahrs;

			///Inverted USBL configuration
			bool inverted_cfg;

			///Assumed time delay to use for AHRS compensation
			double ahrs_delay;
			///TF broadcast listener buffer
			tf2_ros::Buffer buffer;
			///TF broadcast listener
			tf2_ros::TransformListener listener;
			///Transponder map
			std::map<int, std::string> ids;
			///The projection to ENU
			GeographicLib::LocalCartesian proj;

			/// The delay specification
			DelaySpecification delay;
		};
	}
}

/* SEATRAC_NAVLISTENER_H */
#endif
