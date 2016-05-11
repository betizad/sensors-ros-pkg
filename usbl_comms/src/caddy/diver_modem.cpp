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
#include <labust/seatrac/diver_modem.h>
#include <labust/comms/caddy/caddy_messages.h>
#include <labust/comms/caddy/buddy_handler.h>
#include <labust/comms/caddy/surface_handler.h>
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/seatrac_definitions.h>
#include <labust/seatrac/mediator.h>
#include <labust/tools/packer.h>
#include <labust/math/NumberManipulation.hpp>

#include <pluginlib/class_list_macros.h>

#include <std_msgs/UInt8.h>
#include <sensor_msgs/NavSatFix.h>

#include <string>

using namespace labust::seatrac;
using namespace labust::comms::caddy;

DiverModem::DiverModem()
{
  registrations[DatReceive::CID].push_back(Mediator<DatReceive>::makeCallback(
      boost::bind(&DiverModem::onData,this,_1)));
}

DiverModem::~DiverModem(){}

bool DiverModem::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
  // Agent handler functions
  handlers[BUDDY_ID] = boost::bind(&DiverModem::onBuddyData, this, _1);
  handlers[SURFACE_ID] = boost::bind(&DiverModem::onSurfaceData, this, _1);

  // Incoming ROS message handlers
  init.configure(nh, ph);
  nav.configure(nh, ph);
  command.configure(nh, ph);
  chat.configure(nh, ph);

  // Incoming ACOUSTIC message handlers
  buddyhandler.configure(nh, ph);
  surfacehandler.configure(nh, ph);

  // Register trigger for message assembly
  nav.registerTrigger(boost::bind(&DiverModem::assembleMessage, this));

  return true;
}

void DiverModem::onData(const labust::seatrac::DatReceive& msg)
{
  HandlerMap::iterator it=handlers.find(msg.acofix.src);
  if (it != handlers.end())
  {
    handlers[msg.acofix.src](msg.data);
  }
  else
  {
    ROS_WARN("No acoustic data handler found in DiverModem for ID=%d.",msg.acofix.src);
  }
}

void DiverModem::onBuddyData(const std::vector<uint8_t>& data)
{
  BuddyReport message;
  if (!labust::tools::decodePackable(data, &message))
  {
    ROS_WARN("BuddyHandler: Wrong message received from modem.");
    return;
  }

  init.updateInit(message);
  buddyhandler(message, init.offset());

  //Confirmation for commands
  command.currentStatus(message.mission_status);
}

void DiverModem::onSurfaceData(const std::vector<uint8_t>& data)
{
  SurfaceReport message;
  if (!labust::tools::decodePackable(data, &message))
  {
    ROS_WARN("SurfaceHandler: Wrong message received from modem.");
    return;
  }

  if (message.is_master) init.updateInit(message);
  surfacehandler(message, init.offset());
}


void DiverModem::assembleMessage()
{
  DiverReport report;
  nav.updateReport(report, init.offset());
  chat.updateReport(report);
  command.updateReport(report, init.offset());

  //TODO: Determine if chat will actually be sent
  DatQueueClearCmd::Ptr clr(new DatQueueClearCmd());
  DatQueueSetCmd::Ptr cmd(new DatQueueSetCmd());
  SeatracMessage::DataBuffer buf;
  cmd->dest = labust::seatrac::BEACON_ALL;
  labust::tools::encodePackable(report, &buf);
  cmd->data.assign(buf.begin(), buf.end());

  if (!sender.empty())
  {
    sender(clr);
    sender(cmd);
  }
}

PLUGINLIB_EXPORT_CLASS(labust::seatrac::DiverModem, labust::seatrac::DeviceController)
