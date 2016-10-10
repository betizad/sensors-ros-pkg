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
#ifndef USBL_COMMS_SURFACE_USBL_H
#define USBL_COMMS_SURFACE_USBL_H
#include <labust/seatrac/device_controller.h>
#include <labust/seatrac/pinger.h>
#include <labust/seatrac/seatrac_messages.h>

#include <labust/comms/caddy/buddy_handler.h>
#include <labust/comms/caddy/caddy_messages.h>
#include <labust/comms/caddy/diver_handler.h>
#include <labust/seatrac/chat_module.h>
#include <labust/seatrac/command_module.h>
#include <labust/seatrac/init_module.h>
#include <labust/seatrac/nav_module.h>

#include <auv_msgs/NavSts.h>
#include <caddy_msgs/LawnmowerReq.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include <boost/thread/mutex.hpp>
#include <map>
#include <queue>

using labust::comms::caddy::BuddyHandler;
using labust::comms::caddy::DiverHandler;

namespace labust
{
namespace seatrac
{
/**
 * The class implements the status publisher and decoder.
 */
class SurfaceUSBL : virtual public DeviceController
{
  enum
  {
    TIMEOUT = 8,
    BUDDY_ID = 3,
    DIVER_ID = 2,
    SURFACE_ID = 1
  };
  enum
  {
    BUDDY = 0,
    DIVER,
    AGENT_CNT
  };
  typedef boost::function<void(const std::vector<uint8_t>& msg)> Functor;
  typedef std::map<int, Functor> HandlerMap;

public:
  /// Main constructor
  SurfaceUSBL();
  /// Default destructor
  ~SurfaceUSBL();

  /// Listener configuration.
  bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph);

private:
  // Handler the mode chaning topics.
  void onModeChange(const std_msgs::Bool::ConstPtr& msg);
  /// Handles incoming acoustic data.
  void onData(const labust::seatrac::DatReceive& data);
  /// Handles the Buddy data
  void onBuddyData(const std::vector<uint8_t>& data);
  /// Handles the Diver data
  void onDiverData(const std::vector<uint8_t>& data);
  /// Helper function for chat payload processing.
  void updateChat();
  /// Helper method for message assembly in passive mode.
  void assembleMessage();
  /// Helper method to setup and start pinging
  void startPinging(bool flag = true);
  /// Helper method to reset the initialization.
  void resetInit();
  /// The main pinging function
  void run();

  /// The data handlers
  HandlerMap handlers;

  /// The initialization module.
  InitModule init;
  /// The navigation data module.
  NavModule nav;
  /// The command handler.
  CommandModule command;
  /// The chat transmission module.
  ChatModule chat;

  /// The navigation data handler.
  BuddyHandler buddyhandler;
  /// The diver data handler
  DiverHandler diverhandler;

  /// Mode changing subscriber
  ros::Subscriber mode_sub;

  /// The USBL pinger
  Pinger pinger;
  /// The worker thread
  boost::thread worker;
  /// The message assembly mutex
  boost::mutex message_assembly;
  /// The run flag for the worker thread.
  bool run_flag;
  /// The ping rate
  double ping_rate;
  /// The buddy status flag
  int buddy_status;

  /// Master operation flag.
  bool is_master;

  /// The initialization flags
  bool inited[AGENT_CNT];

  /// The delay specification for the devices.
  labust::seatrac::DelaySpecification delay;
};
}
}

/* USBL_COMMS_SURFACE_USBL_H */
#endif
