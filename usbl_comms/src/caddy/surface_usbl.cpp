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
#include <labust/seatrac/surface_usbl.h>
#include <labust/comms/caddy/caddy_messages.h>
#include <labust/comms/caddy/buddy_handler.h>
#include <labust/comms/caddy/diver_handler.h>
#include <labust/comms/ascii6bit.h>
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
using labust::comms::Ascii6Bit;

SurfaceUSBL::SurfaceUSBL():
    pinger(sender, registrations),
    ping_rate(0),
    is_master(false),
    run_flag(false)
{
	registrations[DatReceive::CID].push_back(Mediator<DatReceive>::makeCallback(
			boost::bind(&SurfaceUSBL::onData,this,_1)));
}

SurfaceUSBL::~SurfaceUSBL()
{
  this->startPinging(false);
}

bool SurfaceUSBL::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
  ph.param("ping_rate", ping_rate, ping_rate);
  ph.param("is_master", is_master, is_master);

  // Agent handler functions
  handlers[BUDDY_ID] = boost::bind(&SurfaceUSBL::onBuddyData, this, _1);
  handlers[DIVER_ID] = boost::bind(&SurfaceUSBL::onDiverData, this, _1);

  // Incoming ROS message handlers
  init.configure(nh, ph);
  nav.configure(nh, ph);
  command.configure(nh, ph);
  chat.configure(nh, ph);

  // Incoming ACOUSTIC message handlers
  buddyhandler.configure(nh, ph);
  diverhandler.configure(nh, ph);

  // Local subscribers
  mode_sub = nh.subscribe("master_mode", 1, &SurfaceUSBL::onModeChange, this);

  // Register trigger for message assembly
  nav.registerTrigger(boost::bind(&SurfaceUSBL::assembleMessage, this));

  // Start pinging the diver
  if (is_master) this->startPinging();

  return true;
}

void SurfaceUSBL::startPinging(bool flag)
{
  if (flag)
  {
    run_flag = true;
    worker = boost::thread(boost::bind(&SurfaceUSBL::run, this));
  }
  else
  {
    run_flag = false;
    worker.join();
  }
}

void SurfaceUSBL::resetInit()
{
  // Reset the init
  for (int i=0; i<AGENT_CNT; ++i) inited[i] = false;
}

void SurfaceUSBL::onModeChange(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data == is_master) return;

  if (msg->data)
  {
    //Was slave but should become master
    is_master = true;
    this->startPinging();
  }
  else
  {
    //Was master but should become slave
    is_master = false;
    this->startPinging(false);
  }
}

void SurfaceUSBL::run()
{
  ros::Rate rate((ping_rate==0)?1:ping_rate);

  bool ping_diver=true;

  while(ros::ok() && run_flag && is_master)
  {
    int idx = DIVER;

    //Create the report
    DatSendCmd::Ptr data(new DatSendCmd());
    data->msg_type = AMsgType::MSG_REQU;
    data->dest = DIVER_ID;

    boost::mutex::scoped_lock l(message_assembly);
    // Assemble the report
    SurfaceReport report;
    // Inform that we are in master mode
    report.is_master = true;

    // Setup init
    if (init.isNewInit()) this->resetInit();
    report.origin_lat = init.llh()(0);
    report.origin_lon = init.llh()(1);
    // Test if init needs to be sent
    report.inited = inited[idx];

    // Setup reports
    if (report.inited)
    {
      nav.updateReport(report, init.offset());
      chat.updateReport(report);
      command.updateReport(report, init.offset());
    }
    // Pack the report for sending
    SeatracMessage::DataBuffer buf;
    labust::tools::encodePackable(report,&buf);
    data->data.assign(buf.begin(),buf.end());
    l.unlock();

    ROS_INFO("Pinging: %d", data->dest);
    if (pinger.send(boost::dynamic_pointer_cast<SeatracMessage>(data), TIMEOUT))
    {
      //  If initialized allow confirmations
      if (inited[idx])
      {
        chat.setConfirmation(true);
      }
      else
      {
        // Last message was initialization and reception is validated
        inited[idx] = true;
      }
    }
    else
    {
      ROS_ERROR("SurfaceUSBL: Message sending failed.");
    }

    if (ping_rate) rate.sleep();
    ping_diver = !ping_diver;
  }
}

void SurfaceUSBL::onData(const labust::seatrac::DatReceive& msg)
{
  HandlerMap::iterator it=handlers.find(msg.acofix.src);
  if (it != handlers.end())
  {
    handlers[msg.acofix.src](msg.data);
  }
  else
  {
    ROS_WARN("No acoustic data handler found in BuddyUSBL for ID=%d.",msg.acofix.src);
  }
}

void SurfaceUSBL::onBuddyData(const std::vector<uint8_t>& data)
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

void SurfaceUSBL::onDiverData(const std::vector<uint8_t>& data)
{
  DiverReport message;
  if (!labust::tools::decodePackable(data, &message))
  {
    ROS_WARN("DiverHandler: Wrong message received from modem.");
    return;
  }

  diverhandler(message, init.offset());
}

void SurfaceUSBL::assembleMessage()
{
  //Use triggered message assembly only if slave
  if (is_master) return;

  // TODO: Check if inited (do not encode stuff before inited)

  boost::mutex::scoped_lock l(message_assembly);
  // Assemble the report
  SurfaceReport report;
  // Inform that we are in master mode
  report.is_master = 0;
  report.inited = 1;

  nav.updateReport(report, init.offset());
  chat.updateReport(report);
  command.updateReport(report, init.offset());

  //TODO: Determine if chat will actually be sent

  // Pack and send
  DatQueueClearCmd::Ptr clr(new DatQueueClearCmd());
  DatQueueSetCmd::Ptr cmd(new DatQueueSetCmd());
  SeatracMessage::DataBuffer buf;
  cmd->dest = labust::seatrac::BEACON_ALL;
  labust::tools::encodePackable(report, &buf);
  cmd->data.assign(buf.begin(), buf.end());
  l.unlock();

  if (!sender.empty())
  {
    sender(clr);
    sender(cmd);
  }
}

/*
void SurfaceUSBL::onData(const labust::seatrac::DatReceive& msg)
{
	HandlerMap::iterator it=handlers.find(msg.acofix.src);
	if (it != handlers.end())
	{
	    enum {TIME_GUARD = 1};
		(*handlers[msg.acofix.src])(msg);
		//Assume chat payload was pulled if the aggregate was NOP and reset aggregate.
		has_next = !((sent_cmd == NOP) && ((ros::Time::now() - last_chat).toSec() > TIME_GUARD));
		sent_cmd = 0;
		//Assume that the mission was transmitted
		this->updateChat();
	}
	else
	{
		ROS_WARN("No acoustic data handler found in SurfaceUSBL for ID=%d.",msg.acofix.src);
	}
}


void SurfaceUSBL::onChat(const std_msgs::String::ConstPtr& msg)
{
  for(int i=0; i<msg->data.size(); ++i) chat_buf.push(Ascii6Bit::to6Bit(msg->data[i]));
  ROS_INFO("Added to chat: %s",msg->data.c_str());
  this->updateChat();
}

void SurfaceUSBL::updateChat()
{
  ///TODO(dnad): extract chat handling into a class (diver, surface to have the same)
  if (!has_next)
  {
    surf.chat.clear();

    enum {MAX_CHAT_BYTES=6};
    int nbytes = chat_buf.size();
    if (nbytes > MAX_CHAT_BYTES) nbytes = MAX_CHAT_BYTES;

    ROS_INFO("Adding: %d",nbytes);

    for(int i=0; i<nbytes; ++i){
      surf.chat.push_back(chat_buf.front());
      chat_buf.pop();
    }
    has_next = true;
    last_chat = ros::Time::now();
  }
}
*/
PLUGINLIB_EXPORT_CLASS(labust::seatrac::SurfaceUSBL, labust::seatrac::DeviceController)
