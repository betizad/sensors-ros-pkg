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
#include <labust/seatrac/buddy_usbl.h>
#include <labust/comms/caddy/surface_handler.h>
#include <labust/comms/caddy/diver_handler.h>
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/seatrac_definitions.h>
#include <labust/seatrac/mediator.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/packer.h>
#include <labust/tools/conversions.hpp>

#include <pluginlib/class_list_macros.h>

#include <string>

using namespace labust::seatrac;
using namespace labust::comms::caddy;

BuddyUSBL::BuddyUSBL():
    pinger(sender, registrations),
	ping_rate(0),
	is_master(false),
	run_flag(false)
{
  registrations[DatReceive::CID].push_back(Mediator<DatReceive>::makeCallback(
      boost::bind(&BuddyUSBL::onData,this,_1)));

  //Initialize arrays
  this->resetInit();
  for (int i=0; i<AGENT_CNT; ++i) unconfirmed_cmd[i] = 0;
}

BuddyUSBL::~BuddyUSBL()
{
  this->startPinging(false);
}

bool BuddyUSBL::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
  ph.param("ping_rate", ping_rate, ping_rate);
  ph.param("is_master", is_master, is_master);

  handlers[SURFACE_ID] = boost::bind(&BuddyUSBL::onSurfaceData, this, _1);
  handlers[DIVER_ID] = boost::bind(&BuddyUSBL::onDiverData, this, _1);

  // Incoming ROS message handlers
  init.configure(nh, ph);
  nav.configure(nh, ph);
  payload.configure(nh, ph);
  status.configure(nh, ph);

  // Incoming ACOUSTIC message handlers
  surfacehandler.configure(nh, ph);
  diverhandler.configure(nh, ph);

  // Local subscribers
  mode_sub = nh.subscribe("master_mode", 1, &BuddyUSBL::onModeChange, this);

  if (is_master) this->startPinging();

  return true;
}

void BuddyUSBL::startPinging(bool flag)
{
  if (flag)
  {
    run_flag = true;
    worker = boost::thread(boost::bind(&BuddyUSBL::run, this));
  }
  else
  {
    run_flag = false;
    worker.join();
  }
}

void BuddyUSBL::resetInit()
{
  // Reset the init
  for (int i=0; i<AGENT_CNT; ++i) inited[i] = false;
}

void BuddyUSBL::onModeChange(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data == is_master) return;

  if (msg->data)
  {
    //Was slave but should become master
    is_master = true;
    //Allow external init determination
    this->startPinging();
  }
  else
  {
    //Was master but should become slave
    is_master = false;
    this->startPinging(false);
  }
}

void BuddyUSBL::run()
{
  ros::Rate rate((ping_rate==0)?1:ping_rate);

  bool ping_diver=true;

  while(ros::ok() && run_flag && is_master)
  {
    int idx = ping_diver?DIVER:SURFACE;

    //Create the report
    DatSendCmd::Ptr data(new DatSendCmd());
    data->msg_type = AMsgType::MSG_REQU;
    data->dest = ping_diver?DIVER_ID:SURFACE_ID;

    boost::mutex::scoped_lock l(message_assembly);
    // Assemble the report
    BuddyReport report;

    // Setup init
    if (init.isNewInit()) this->resetInit();
    report.origin_lat = init.llh()(0);
    report.origin_lon = init.llh()(1);
    report.inited = inited[idx];

    // Setup reports
    if (report.inited)
    {
      nav.updateReport(report, init_offset);
      payload.updateReport(report);
      status.updateReport(report, init_offset);

      // Send Buddy position depending on agent and time
      report.has_position = 1;
      if (idx == SURFACE)
      {
        report.has_position = this->sendPosition(idx);
      }
    }
    else
    {
      ROS_INFO("Not inited: %d", data->dest);
    }
    // Pack the report for sending
    SeatracMessage::DataBuffer buf;
    labust::tools::encodePackable(report,&buf);
    data->data.assign(buf.begin(),buf.end());
    ROS_INFO("First byte encoded as:%d",data->data[0]);
    l.unlock();

    ROS_INFO("Pinging: %d", data->dest);
    if (pinger.send(boost::dynamic_pointer_cast<SeatracMessage>(data), TIMEOUT))
    {
      //  If initialized allow confirmations
      if (inited[idx])
      {
        // Confirm message sending
        if ((unconfirmed_cmd[idx] != 0) &&
            (unconfirmed_cmd[idx] == report.mission_status))
        {
          status.setConfirmation(true);
          unconfirmed_cmd[idx] = 0;
        }
      }
      else
      {
        // Last message was initialization and reception is validated
        inited[idx] = true;
      }
    }
    else
    {
      ROS_ERROR("BuddyUSBL: Message delivery failed.");
    }

    if (ping_rate) rate.sleep();
    ping_diver = !ping_diver;
  }
}

void BuddyUSBL::onData(const labust::seatrac::DatReceive& msg)
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

void BuddyUSBL::onSurfaceData(const std::vector<uint8_t>& data)
{
  SurfaceReport message;
  if (!labust::tools::decodePackable(data, &message))
  {
    ROS_WARN("SurfaceHandler: Wrong message received from modem.");
    return;
  }

  init.updateInit(message);
  surfacehandler(message, init.offset());
  if (message.command) unconfirmed_cmd[SURFACE] = message.command;
}

void BuddyUSBL::onDiverData(const std::vector<uint8_t>& data)
{
  DiverReport message;
  if (!labust::tools::decodePackable(data, &message))
  {
    ROS_WARN("DiverHandler: Wrong message received from modem.");
    return;
  }

  diverhandler(message, init.offset());
  if (message.command) unconfirmed_cmd[SURFACE] = message.command;
}

bool BuddyUSBL::sendPosition(int agent)
{
  //Agents should get buddy position updates atleast every dTmax sec.
  const double dTmax(10.0);

  double dT = (ros::Time::now() - buddy_pos_update[agent]).toSec();
  if (dT > dTmax)
  {
    buddy_pos_update[agent] = ros::Time::now();
    return true;
  }

  return false;
}

PLUGINLIB_EXPORT_CLASS(labust::seatrac::BuddyUSBL, labust::seatrac::DeviceController)
