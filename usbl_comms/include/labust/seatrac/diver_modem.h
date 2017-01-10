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
#ifndef USBL_COMMS_DIVER_MODEM_H
#define USBL_COMMS_DIVER_MODEM_H
#include <labust/comms/caddy/ac_handler.h>
#include <labust/comms/caddy/buddy_handler.h>
#include <labust/comms/caddy/caddy_messages.h>
#include <labust/comms/caddy/surface_handler.h>
#include <labust/seatrac/chat_module.h>
#include <labust/seatrac/command_module.h>
#include <labust/seatrac/device_controller.h>
#include <labust/seatrac/diver_payload.h>
#include <labust/seatrac/init_module.h>
#include <labust/seatrac/nav_module.h>
#include <labust/seatrac/seatrac_messages.h>

#include <auv_msgs/NavSts.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <boost/thread/mutex.hpp>

using labust::comms::caddy::BuddyHandler;
using labust::comms::caddy::SurfaceHandler;

namespace labust
{
namespace seatrac
{
/**
 * The class implements the status publisher and decoder.
 */
class DiverModem : virtual public DeviceController
{
  typedef boost::function<void(const std::vector<uint8_t>& msg)> Functor;
  typedef std::map<int, Functor> HandlerMap;

  enum
  {
    TIMEOUT = 4,
    BUDDY_ID = 3,
    SURFACE_ID = 1
  };

public:
  /// Main constructor
  DiverModem();
  /// Default destructor
  ~DiverModem();

  /// Listener configuration.
  bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph);

private:
  /// Helper method for message assembly.
  void assembleMessage();
  /// Handles incoming acoustic data.
  void onData(const labust::seatrac::DatReceive& data);
  /// Handles the Buddy data
  void onBuddyData(const std::vector<uint8_t>& data);
  /// Handles the Surface data
  void onSurfaceData(const std::vector<uint8_t>& data);

  /// Handlers for acoustic messages
  HandlerMap handlers;

  /// The initialization module.
  InitModule init;
  /// The navigation data module.
  NavModule nav;
  /// The command handler.
  CommandModule command;
  /// The chat transmission module.
  ChatModule chat;
  /// The Diver payload handling.
  DiverPayload payload;

  /// The Buddy data handler.
  BuddyHandler buddyhandler;
  /// The surface data handler.
  SurfaceHandler surfacehandler;
  /// The surface master flag
  bool surface_master;
  /// Last time the full payload was sent.
  ros::Time last_payload;

  /// The delay specification for the devices.
  labust::seatrac::DelaySpecification delay;
};
}
}

/* USBL_COMMS_DIVER_MODEM_H */
#endif
