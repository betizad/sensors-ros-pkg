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
#ifndef USBL_COMMS_DIVER_HANDLER_H
#define USBL_COMMS_DIVER_HANDLER_H
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
      ///Class for handling Diver acoustic messages and publish them to ROS.
      class DiverHandler
      {
      public:
        ///Main constructor
        DiverHandler():command("diver", true), chat("diver"){};

        bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph);

        void operator()(const DiverReport& message, const Eigen::Vector3d& offset);

      protected:
        // Method for handling the navigation part.
        void navHandler(const DiverReport& message, const Eigen::Vector3d& offset);
        // Method for handling the navigation part.
        void payloadHandler(const DiverReport& message);

        //Diver navigation data publisher
        ros::Publisher nav_pub;
        //Diver bio-info publisher
        ros::Publisher payload_pub;
        // The common chat handler
        labust::seatrac::ChatHandler chat;
        // The common command handler
        labust::seatrac::StatusHandler command;
      };
    }
  }
}
/* USBL_COMMS_DIVER_HANDLER_H */
#endif



