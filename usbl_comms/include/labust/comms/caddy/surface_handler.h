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
 *  Author: Dula Nad
 *  Created: 05.03.2015.
 *********************************************************************/
#ifndef USBL_COMMS_SURFACE_HANDLER_H
#define USBL_COMMS_SURFACE_HANDLER_H
#include <labust/comms/caddy/caddy_messages.h>
#include <labust/seatrac/chat_handler.h>
#include <labust/seatrac/status_handler.h>

#include <ros/ros.h>

#include <Eigen/Dense>

#include <cstdint>

namespace labust
{
	namespace comms
	{
		namespace caddy
		{
			///Class for handling Surface acoustic messages and publish them to ROS.
			class SurfaceHandler
			{
			  enum {n=0,e,d};
			public:
				///Main constructor
				SurfaceHandler():command("surface", true){};

				bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph);

				void operator()(const SurfaceReport& message, const Eigen::Vector3d& offset);

			protected:
                // Method for handling the navigation part.
                void navHandler(const SurfaceReport& message, const Eigen::Vector3d& offset);

				// Surface navigation data publisher
				ros::Publisher nav_pub;
				// Surface diver navigation publisher
				ros::Publisher diverpos_pub;
                /// The initialization position publisher
                ros::Publisher init_pub;
                /// The common chat handler
                labust::seatrac::ChatHandler chat;
                /// The common status/command handler
                labust::seatrac::StatusHandler command;
			};
		}
	}
}
/* USBL_COMMS_SURFACE_HANDLER_H */
#endif



