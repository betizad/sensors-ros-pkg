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
 *  Author : Dula Nad
 *  Created: 23.01.2013.
 *********************************************************************/
#include <labust/seatrac/seatrac_definitions.h>
#include <labust/seatrac/seatrac_factory.h>
#include <labust/seatrac/seatrac_sim.h>
#include <pluginlib/class_list_macros.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/conversions.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <underwater_msgs/AcSimRegister.h>

#include <labust/tools/StringUtilities.hpp>

using labust::seatrac::SeatracSim;

SeatracSim::SeatracSim()
  : state(IDLE)
  , expected_id(0)
  , node_id(1)
  , max_distance(500.0)
  , vos(1500)
  , time_overhead(0.1)
  , is_modem(false)
  , bps(100)
  , sim_frame_id("base_link_sim")
  , registered(false)
  , max_data_duration(1.5)
  , internal_ahrs(false)
  , surface_depth(-1.0)
  , listener(buffer)
{
}

SeatracSim::~SeatracSim()
{
  sleeper.stop();
  unregisterModem();
}

bool SeatracSim::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
  delay.configureSim(ph);
  ph.param("sim_node_id", node_id, node_id);
  ph.param("sim_max_distance", max_distance, max_distance);
  ph.param("sim_vos", vos, vos);
  ph.param("sim_time_overhead", time_overhead, time_overhead);
  ph.param("sim_is_modem", is_modem, is_modem);
  ph.param("sim_max_data_duration", max_data_duration, max_data_duration);
  ph.param("sim_bps", bps, bps);
  ph.param("use_ahrs", internal_ahrs, internal_ahrs);
  ph.param("sim_surface_depth", surface_depth, surface_depth);
  ph.param("sim_frame_id", sim_frame_id, sim_frame_id);
  std::string key;
  if (nh.searchParam("tf_prefix", key))
    nh.getParam(key, tf_prefix);

  std::vector<double> offset(3, 0), orot(3, 0);
  ph.param("sim_offset", offset, offset);
  ph.param("sim_orot", orot, orot);
  this->offset << offset[0], offset[1], offset[2];
  labust::tools::quaternionFromEulerZYX(M_PI * orot[0] / 180,
                                        M_PI * orot[1] / 180,
                                        M_PI * orot[2] / 180, this->orot);

  navsts =
      nh.subscribe<auv_msgs::NavSts>("navsts", 1, &SeatracSim::onNavSts, this);
  medium_in = nh.subscribe<underwater_msgs::MediumTransmission>(
      "medium_in", 16, &SeatracSim::onMediumTransmission, this);
  unregister_sub = nh.subscribe<std_msgs::Bool>(
      "unregister_modems", 1, &SeatracSim::onUnregisterModem, this);
  medium_out =
      nh.advertise<underwater_msgs::MediumTransmission>("medium_out", 1);
  registered_nodes = nh.subscribe<std_msgs::Int32MultiArray>(
      "registered_nodes", 16, &SeatracSim::onRegisteredNodes, this);

  // Create timer
  sleeper = nh.createTimer(ros::Duration(delay.ping_duration),
                           &SeatracSim::onUSBLTimeout, this, true, false);

  // Register to medium with node id, navsts topic name, etc.
  unregisterModem();
  registerModem();

  return true;
}

void SeatracSim::registerModem()
{
  if (registered)
    return;

  ROS_INFO("SeatracSim: registering modem to medium.");

  ros::NodeHandle nh;
  ros::ServiceClient reg =
      nh.serviceClient<underwater_msgs::AcSimRegister>("register_modem");

  if (reg.waitForExistence(ros::Duration(5.0)))
  {
    underwater_msgs::AcSimRegister srv;
    srv.request.node_id = node_id;
    srv.request.navsts_topic = navsts.getTopic();
    registered = reg.call(srv);
  }

  if (!registered)
  {
    ROS_WARN("SeatracSim: modem is not registered with the acoustic medium.");
  }
}

void SeatracSim::unregisterModem()
{
  ROS_INFO("SeatracSim: deregistering modem from medium.");

  ros::NodeHandle nh;
  ros::ServiceClient reg =
      nh.serviceClient<underwater_msgs::AcSimRegister>("unregister_modem");

  if (reg.waitForExistence(ros::Duration(5.0)))
  {
    underwater_msgs::AcSimRegister srv;
    srv.request.node_id = node_id;
    srv.request.navsts_topic = navsts.getTopic();
    registered = !reg.call(srv);
  }

  if (!registered)
  {
    ROS_INFO("SeatracSim: unregistered modem from acoustic medium.");
  }
}

void SeatracSim::onRegisteredNodes(
    const std_msgs::Int32MultiArray::ConstPtr& msg)
{
  for (int i = 0; i < msg->data.size(); ++i)
    node_list.insert(msg->data[i]);
}

bool SeatracSim::nodeExists(int node_id)
{
  // Backward compatibility
  if (node_list.empty())
    return true;

  return node_list.find(node_id) != node_list.end();
}

void SeatracSim::onNavSts(const auv_msgs::NavSts::ConstPtr& msg)
{
  boost::shared_ptr<StatusResp> resp(new StatusResp());

  // \todo switch to USBL frame to incorporate offsets
  // Setup flags
  resp->status.status_output.ACC_CAL = 0;
  resp->status.status_output.AHRS_COMP_DATA = 0;
  resp->status.status_output.AHRS_RAW_DATA = 0;
  resp->status.status_output.ATTITUDE = 1;
  resp->status.status_output.ENVIRONMENT = 0;
  resp->status.status_output.MAG_CAL = 0;

  resp->status.attitude[Status::ROLL] = msg->orientation.roll * Status::ATT_SC;
  resp->status.attitude[Status::PITCH] =
      msg->orientation.pitch * Status::ATT_SC;
  resp->status.attitude[Status::YAW] = msg->orientation.yaw * Status::ATT_SC;

  Eigen::Vector3d offset_ned(0, 0, 0);
  try
  {
    geometry_msgs::TransformStamped transformLocal = buffer.lookupTransform(
        msg->header.frame_id, tf_prefix + sim_frame_id, ros::Time(0));
    Eigen::Quaterniond rot(transformLocal.transform.rotation.w,
                           transformLocal.transform.rotation.x,
                           transformLocal.transform.rotation.y,
                           transformLocal.transform.rotation.z);

    offset_ned = rot.matrix() * offset;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }

  // Set the current node state
  boost::mutex::scoped_lock ls(position_mux);
  navstate = *msg;
  // TODO: apply also rotation in future
  navstate.position.north += offset_ned(0);
  navstate.position.east += offset_ned(1);
  navstate.position.depth += offset_ned(2);
  navstate_delayed = navstate;
  ls.unlock();

  try
  {
    geometry_msgs::TransformStamped transformLocal = buffer.lookupTransform(
        msg->header.frame_id, tf_prefix + sim_frame_id,
        ros::Time(msg->header.stamp -
                  ros::Duration(delay.usbl_processing_duration)));
    Eigen::Quaterniond rot(transformLocal.transform.rotation.w,
                           transformLocal.transform.rotation.x,
                           transformLocal.transform.rotation.y,
                           transformLocal.transform.rotation.z);

    offset_ned = rot.matrix() * offset;

    ls.lock();
    // TODO: apply also rotation in future
    navstate_delayed.position.north =
        transformLocal.transform.translation.x + offset_ned(0);
    navstate_delayed.position.east =
        transformLocal.transform.translation.y + offset_ned(1);
    navstate_delayed.position.depth =
        transformLocal.transform.translation.z + offset_ned(2);
    double roll, pitch, yaw;
    labust::tools::eulerZYXFromQuaternion(transformLocal.transform.rotation,
                                          roll, pitch, yaw);
    navstate_delayed.orientation.roll = roll;
    navstate_delayed.orientation.pitch = pitch;
    navstate_delayed.orientation.yaw = yaw;
    ls.unlock();
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }

  this->sendMessage(boost::dynamic_pointer_cast<SeatracMessage const>(resp));
}

bool SeatracSim::send(const SeatracMessage::ConstPtr& msg)
{
  // Try registering before sending
  this->registerModem();
  if (!registered)
  {
    this->unregisterModem();
    return false;
  }

  if (getState() != IDLE)
  {
    // Reply with device busy
    ROS_DEBUG("SeatracSim: Modem is busy.");
    sendError<PingSendResp>(CST::XCVR::BUSY);
    return true;
  }

  // Inhibit sending if above surface depth
  if (this->navstate.position.depth < surface_depth)
  {
    ROS_WARN("Modem is on surface so it can not be heard by anyone.");
    return true;
  }

  underwater_msgs::MediumTransmission::Ptr tomedium(
      new underwater_msgs::MediumTransmission());
  tomedium->header.stamp = ros::Time::now();
  tomedium->sender = this->node_id;
  tomedium->position = navstate;
  // Broadcast by default
  tomedium->receiver = 0;
  // Minimum duration is ping
  tomedium->duration = delay.ping_duration;
  // Minimum data duration is 0
  double data_duration(0);
  // Total timeout for ping
  double total_timeout(delay.ping_duration);

  if (msg->getCid() == PingSendCmd::CID)
  {
    PingSendCmd::ConstPtr cmd =
        boost::dynamic_pointer_cast<PingSendCmd const>(msg);
    tomedium->receiver = cmd->dest;
    expected_id = cmd->dest;
    // Check timeout
    if (this->nodeExists(tomedium->receiver))
      total_timeout +=
          delay.ping_reply_duration + delay.usbl_processing_duration;

    ROS_DEBUG("SeatracSim: Sending ping (%d->%d).", node_id, expected_id);
    this->setState(WAIT_PING_REPLY);
  }
  else if (msg->getCid() == DatSendCmd::CID)
  {
    DatSendCmd::ConstPtr cmd =
        boost::dynamic_pointer_cast<DatSendCmd const>(msg);
    tomedium->receiver = cmd->dest;
    expected_id = cmd->dest;
    if ((cmd->msg_type == AMsgType::MSG_REQX) ||
        (cmd->msg_type == AMsgType::MSG_REQU) ||
        (cmd->msg_type == AMsgType::MSG_OWAYU))
    {
      ROS_DEBUG("SeatracSim: Sending data with USBL ping (%d->%d).", node_id,
                expected_id);
      tomedium->duration += 8 * cmd->data.size() / bps;
      // The 2*ping payload + data sent + the maximum amount of data that can
      // be returned
      total_timeout += 8 * cmd->data.size() / bps;
      // Add the maximum return data time if node exists
      if (this->nodeExists(tomedium->receiver))
        total_timeout += delay.ping_reply_duration +
                         delay.usbl_processing_duration + max_data_duration;
    }
    else
    {
      // No ping, no USBL info; pure data
      ROS_DEBUG("SeatracSim: Sending only data (%d->%d).", node_id,
                expected_id);
      tomedium->duration = 8 * cmd->data.size() / bps;
      // The data sent
      total_timeout = 8 * cmd->data.size() / bps;
      // Add the maximum return data time if node exists
      if (this->nodeExists(tomedium->receiver))
        total_timeout += max_data_duration;
    }

    this->setState(WAIT_DATA_REPLY);
  }
  else if (msg->getCid() == DatQueueSetCmd::CID)
  {
    // \todo Send back acknowledgement ?
    boost::mutex::scoped_lock l(reply_queue_mux);
    DatQueueSetCmd::ConstPtr dat =
        boost::dynamic_pointer_cast<DatQueueSetCmd const>(msg);
    DatSendCmd::Ptr data(new DatSendCmd());
    data->dest = dat->dest;
    data->data = dat->data;
    data->msg_type = AMsgType::MSG_OWAY;
    reply_queue.push(data);
    // ROS_INFO("Queue size: %d",reply_queue.size());
    return true;
  }
  else if (msg->getCid() == DatQueueClearCmd::CID)
  {
    // \todo Send back acknowledgement ?
    boost::mutex::scoped_lock l(reply_queue_mux);
    while (!reply_queue.empty())
      reply_queue.pop();
    return true;
  }
  else
  {
    ROS_WARN("SeatracSim: No send handler implemented for CID=0x%x [%s].",
             msg->getCid(),
             SeatracFactory::getResponseName(msg->getCid()).c_str());
    return false;
  }

  // Same for all valid messages
  // Pack ping and send
  SeatracFactory::encodePacket(msg, &tomedium->message);
  this->sendToMedium(tomedium);
  if (this->nodeExists(tomedium->receiver))
  {
    // Standard wait time if node exists (Transmission time + distance + safety
    // overhead)
    this->startTimer(total_timeout + 2 * max_distance / vos + time_overhead);
  }
  else
  {
    // Only send time + range timeout
    this->startTimer(total_timeout + 2 * max_distance / vos + time_overhead);
  }

  return true;
}

bool SeatracSim::resend()
{
  ROS_WARN("SeatracSim: Resend not implemented.");
  return true;
}

void SeatracSim::onMediumTransmission(
    const underwater_msgs::MediumTransmission::ConstPtr& msg)
{
  // Ignore self-messages - there should be no messages
  if (msg->sender == node_id)
    return;
  // Ignore message not meant to be heard by this node
  if (msg->listener_id != node_id)
    return;
  // Inhibit sending if above surface depth
  if (this->navstate.position.depth < surface_depth)
  {
    ROS_WARN("Modem is on surface so it can not hear anything.");
    return;
  }

  // Test if it is possible to hear the message

  // Throw dice if we will have a CRC error

  // Unpack message
  SeatracMessage::Ptr message;
  if (SeatracFactory::decodePacket(msg->message, message))
  {
    // Dispatch message based on CID (PING, DATA)
    // If more message types should be handled switch to map dispatch for
    // readability.
    if (message->getCid() == PingSendCmd::CID)
    {
      processPingCmd(msg);
    }
    else if (message->getCid() == DatSendCmd::CID)
    {
      processDataCmd(msg, *boost::dynamic_pointer_cast<DatSendCmd>(message));
    }
    else
    {
      ROS_WARN("No receive handler for CID=0x%x [%s]", message->getCid(),
               SeatracFactory::getResponseName(message->getCid()).c_str());
    }
  }
}

void SeatracSim::processPingCmd(
    const underwater_msgs::MediumTransmission::ConstPtr& msg)
{
  SeatracMessage::ConstPtr out;
  int state = getState();
  ROS_DEBUG("SeatracSim: Processing arrived PingCmd sender=%d, receiver=%d. "
            "Current state is ID=%d.",
            msg->sender, msg->receiver, state);

  if (state == IDLE)
  {
    // Reply on ping command
    if (msg->receiver == node_id)
    {
      ROS_DEBUG("SeatracSim: Reply on ping (%d->%d).", msg->sender, node_id);
      underwater_msgs::MediumTransmission::Ptr rep(
          new underwater_msgs::MediumTransmission());
      rep->sender = node_id;
      rep->receiver = msg->sender;
      rep->duration = delay.ping_reply_duration;
      // Add the paket processing duration
      rep->duration += delay.usbl_processing_duration;
      rep->position = navstate;
      PingSendCmd::Ptr repcmd(new PingSendCmd());
      SeatracFactory::encodePacket(repcmd, &rep->message);
      this->sendToMedium(rep);
    }

    // Create PING_REQ message and send to callback
    // TODO(dnad) Add for USBL the azimuth measurement on listening
    PingReq::Ptr req(new PingReq());
    fillPosReply(req, msg, true);
    req->acofix.src = msg->sender;
    req->acofix.dest = msg->receiver;
    /*req->acofix.flags.POSITION_VALID = 0;
    req->acofix.flags.RANGE_VALID = 0;
    req->acofix.flags.USBL_VALID = 0;
    if (!is_modem)
    {
      req->acofix.flags.USBL_VALID = 1;
      req->acofix.usbl.azimuth = msg->azimuth*AcoFix::ANGLE_SC;
      req->acofix.usbl.elevation = msg->elevation*AcoFix::ANGLE_SC;
    }
    this->fillAcoFix(req->acofix);*/
    out = req;
  }
  else if (state == WAIT_PING_REPLY)
  {
    if (msg->sender == expected_id)
    {
      ROS_DEBUG("SeatracSim: Received ping reply (%d->%d).", node_id,
                expected_id);
      // Stop timeout waiting condition.
      sleeper.stop();
      // Handle ping reply
      // Create the PING_RESP and send navigation data
      PingResp::Ptr resp(new PingResp());
      // \todo Set only range for modems
      fillPosReply(resp, msg);
      resp->acofix.dest = node_id;
      resp->acofix.src = expected_id;
      /*this->fillAcoFix(resp->acofix);
        resp->acofix.flags.RANGE_VALID = 1;
      resp->acofix.flags.USBL_VALID = 0;
      resp->acofix.flags.POSITION_VALID = 0;
      resp->acofix.range.dist = msg->range*AcoFix::RANGE_SC;

      if (!is_modem)
      {
        resp->acofix.flags.USBL_VALID = 1;
        resp->acofix.flags.POSITION_VALID = 1;
        resp->acofix.usbl.azimuth = msg->azimuth*AcoFix::ANGLE_SC;
        resp->acofix.usbl.elevation = msg->elevation*AcoFix::ANGLE_SC;
        boost::mutex::scoped_lock l(position_mux);
        resp->acofix.position[AcoFix::x] = (msg->position.position.north -
      navstate.position.north)*AcoFix::RANGE_SC;
        resp->acofix.position[AcoFix::y] = (msg->position.position.east -
      navstate.position.east)*AcoFix::RANGE_SC;
        resp->acofix.position[AcoFix::z] = (msg->position.position.depth -
      navstate.position.depth)*AcoFix::RANGE_SC;
      }*/

      out = resp;
    }
    else
    {
      ROS_WARN("SeatracSim: Waited for ping reply (%d->%d). Received ping cmd "
               "from %d.",
               node_id, expected_id, msg->sender);
      // Send PING_ERROR with XCVR_RESP_WRONG
      PingError::Ptr err(new PingError());
      err->beacon_id = expected_id;
      err->status = CST::XCVR::RESP_WRONG;
      out = err;
    }
  }
  this->setState(IDLE);
  if (out != 0)
    this->sendMessage(out);
}

template <class MsgType>
void SeatracSim::fillPosReply(
    MsgType& resp, const underwater_msgs::MediumTransmission::ConstPtr& msg,
    bool passive)
{
  // \todo Set only range for modems
  this->fillAcoFix(resp->acofix);
  boost::mutex::scoped_lock l(position_mux);
  double dn = msg->position.position.north - navstate_delayed.position.north;
  double de = msg->position.position.east - navstate_delayed.position.east;
  double dd = msg->position.position.depth - navstate_delayed.position.depth;
  //  ROS_ERROR("Position other: %f %f %f",
  //    msg->position.position.north,
  //    msg->position.position.east,
  //    msg->position.position.depth);
  //  ROS_ERROR("Position : %f %f %f",
  //    navstate.position.north,
  //    navstate.position.east,
  //    navstate.position.depth);
  l.unlock();

  // resp->acofix.dest = node_id;
  // resp->acofix.src = expected_id;
  resp->acofix.flags.RANGE_VALID = !passive;
  resp->acofix.flags.USBL_VALID = 0;
  resp->acofix.flags.POSITION_VALID = 0;
  resp->acofix.range.dist =
      sqrt(dn * dn + de * de + dd * dd) * AcoFix::RANGE_SC;

  if (!is_modem)
  {
    resp->acofix.flags.USBL_VALID = 1;
    resp->acofix.flags.POSITION_VALID = !passive;

    double rh = sqrt(dn * dn + de * de);
    double el(0);
    if (rh == 0)
      el = 90 * (dd >= 0 ? 1 : -1);
    else
      el = 180 * atan(dd / rh) / M_PI;
    double az = atan2(de, dn);
    if (internal_ahrs)
      az -= navstate_delayed.orientation.yaw;
    az = 180 * labust::math::wrapRad(az) / M_PI;
    if (az < 0)
      az += 360;
    resp->acofix.usbl.azimuth = az * AcoFix::ANGLE_SC;
    resp->acofix.usbl.elevation = el * AcoFix::ANGLE_SC;
    if (!passive)
    {
      resp->acofix.position[AcoFix::x] = dn * AcoFix::RANGE_SC;
      resp->acofix.position[AcoFix::y] = de * AcoFix::RANGE_SC;
      resp->acofix.position[AcoFix::z] = dd * AcoFix::RANGE_SC;
    }
  }

  resp->acofix.vos = vos * AcoFix::RANGE_SC;
}

void SeatracSim::processDataCmd(
    const underwater_msgs::MediumTransmission::ConstPtr& msg,
    const DatSendCmd& incoming)
{
  SeatracMessage::ConstPtr out;
  int state = getState();
  ROS_DEBUG("SeatracSim: Processing arrived DatCmd (%d->%d). Current state is "
            "ID=%d.",
            msg->sender, msg->receiver, state);

  if (state == IDLE)
  {
    // Look into data command
    if (msg->receiver == node_id)
    {
      ROS_INFO("SeatracSim: Reply on DatCmd (%d->%d).", msg->sender, node_id);
      if ((incoming.msg_type == AMsgType::MSG_REQU) ||
          (incoming.msg_type == AMsgType::MSG_REQX))
      {
        underwater_msgs::MediumTransmission::Ptr rep(
            new underwater_msgs::MediumTransmission());
        rep->sender = node_id;
        rep->receiver = msg->sender;
        rep->duration = delay.ping_reply_duration;
        // Add the paket processing duration
        rep->duration += delay.usbl_processing_duration;
        rep->position = navstate;

        // If nothing to reply, return only the pinging part of data
        DatSendCmd::Ptr repcmd(new DatSendCmd());
        boost::mutex::scoped_lock l(reply_queue_mux);
        if (!reply_queue.empty())
        {
          DatSendCmd::Ptr mr = reply_queue.front();
          if ((msg->sender == mr->dest) || (mr->dest == BEACON_ALL))
          {
            repcmd = mr;
            reply_queue.pop();
            rep->duration += 8 * repcmd->data.size() / bps;
          }
        }
        l.unlock();
        SeatracFactory::encodePacket(repcmd, &rep->message);

        this->sendToMedium(rep);
      }
    }

    // Create DAT_RECEIVE message and send to callback
    // TODO(dnad) Add for USBL the azimuth measurement on listening
    DatReceive::Ptr req(new DatReceive());
    req->data = incoming.data;
    fillPosReply(req, msg, true);
    req->acofix.src = msg->sender;
    req->acofix.dest = msg->receiver;
    /*req->acofix.flags.POSITION_VALID = 0;
    req->acofix.flags.RANGE_VALID = 0;
    req->acofix.flags.USBL_VALID = 0;

    if (!is_modem)
    {
      req->acofix.flags.USBL_VALID = 1;
      req->acofix.usbl.azimuth = msg->azimuth*AcoFix::ANGLE_SC;
      req->acofix.usbl.elevation = msg->elevation*AcoFix::ANGLE_SC;
    }
    this->fillAcoFix(req->acofix);*/
    out = req;
  }
  else if (state == WAIT_DATA_REPLY)
  {
    if (msg->sender == expected_id)
    {
      ROS_DEBUG("SeatracSim: Received data reply (%d->%d).", node_id,
                expected_id);
      // Stop timeout waiting condition.
      sleeper.stop();
      // Handle ping reply
      // Create the PING_RESP and send navigation data
      DatReceive::Ptr resp(new DatReceive());
      // \todo Set only range for modems
      resp->data = incoming.data;
      fillPosReply(resp, msg);
      resp->acofix.src = expected_id;
      resp->acofix.dest = node_id;
      /*
      resp->acofix.flags.RANGE_VALID = 1;
      resp->acofix.flags.USBL_VALID = 0;
      resp->acofix.flags.POSITION_VALID = 0;
      resp->acofix.range.dist = msg->range*AcoFix::RANGE_SC;
      this->fillAcoFix(resp->acofix);

      if (!is_modem)
      {
        resp->acofix.flags.USBL_VALID = 1;
        resp->acofix.flags.POSITION_VALID = 1;
        resp->acofix.usbl.azimuth = msg->azimuth*AcoFix::ANGLE_SC;
        resp->acofix.usbl.elevation = msg->elevation*AcoFix::ANGLE_SC;
        boost::mutex::scoped_lock l(position_mux);
        resp->acofix.position[AcoFix::x] = (msg->position.position.north -
      navstate.position.north)*AcoFix::RANGE_SC;
        resp->acofix.position[AcoFix::y] = (msg->position.position.east -
      navstate.position.east)*AcoFix::RANGE_SC;
        resp->acofix.position[AcoFix::z] = (msg->position.position.depth -
      navstate.position.depth)*AcoFix::RANGE_SC;
      }*/
      out = resp;
    }
    else
    {
      ROS_WARN("SeatracSim: Waited for data cmd reply (%d->%d). Received data "
               "cmd from %d.",
               node_id, expected_id, msg->sender);
      // Send DAT_ERROR with XCVR_RESP_WRONG
      DatError::Ptr err(new DatError());
      err->beacon_id = expected_id;
      err->status = CST::XCVR::RESP_WRONG;
      out = err;
    }
  }
  this->setState(IDLE);
  if (out != 0)
    this->sendMessage(out);
}

void SeatracSim::onUSBLTimeout(const ros::TimerEvent& e)
{
  ROS_WARN("SeatracSim: USBL timeout.");

  int state = getState();

  if (state == WAIT_PING_REPLY)
  {
    sendError<PingError>(CST::XCVR::RESP_TIMEOUT);
  }
  else if (state == WAIT_DATA_REPLY)
  {
    sendError<DatError>(CST::XCVR::RESP_TIMEOUT);
  }

  this->setState(IDLE);
}

PLUGINLIB_EXPORT_CLASS(labust::seatrac::SeatracSim,
                       labust::seatrac::SeatracComms)
