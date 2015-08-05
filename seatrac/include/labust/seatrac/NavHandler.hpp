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
#ifndef NAVHANDLER_HPP_
#define NAVHANDLER_HPP_
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>

#include <map>

namespace labust
{
	namespace seatrac
	{
		/**
		 * The class implements the navigation handler and publisher.
		 * \todo Add TF broadcast, etc.
		 */
		class NavHandler
		{
			enum {east=0, north, depth};
		public:
			/**
			 * Main constructor
			 */
			NavHandler();
			/**
			 * Initialize and setup controller.
			 */
			void onInit();

			/**
			 * Main handling operator.
			 */
			void operator()(int type, std::vector<uint8_t>& payload);

		private:

			void onExDepth(const std_msgs::Float32::ConstPtr& depth)
			{
				exDepth = depth->data;
			}

			///Depth aiding subscription
			ros::Subscriber depthAiding;
			///Position fix publisher
			ros::Publisher usblFix;
			///NavSts publisher for debugging
			ros::Publisher position;
			///Depth aided NavSts publisher for debugging
			ros::Publisher positionDA;
			///Relative position publisher for backward compatibility
			ros::Publisher relativePos;
			///TF2 transform buffer
			tf2_ros::Buffer buffer;
			///TF2 transform listener
			tf2_ros::TransformListener listener;
			///Remote depth variable
			double exDepth;
			///Use vehicle AHRS
			bool useVehicleAHRS;
		};
	}
}

/* STATUSHANDLER_HPP_ */
#endif
