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
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/seatrac_definitions.h>
#include <labust/seatrac/mediator.h>
#include <labust/seatrac/queued_pinger.h>
#include <pluginlib/class_list_macros.h>

#include <string>

using namespace labust::seatrac;

QueuedPinger::QueuedPinger():
	enhanced_usbl(false),
	enhanced_data(false),
	is_busy(false),
	timeout(2.0)
{
	registrations[PingError::CID].push_back(boost::bind(&QueuedPinger::onPingErrors,this,_1));
	registrations[PingSendResp::CID].push_back(boost::bind(&QueuedPinger::onPingErrors,this,_1));
	registrations[PingResp::CID].push_back(boost::bind(&QueuedPinger::onPingReplies,this,_1));
	registrations[DatError::CID].push_back(boost::bind(&QueuedPinger::onPingErrors,this,_1));
	registrations[DatSendResp::CID].push_back(boost::bind(&QueuedPinger::onPingErrors,this,_1));
	registrations[DatReceive::CID].push_back(boost::bind(&QueuedPinger::onPingReplies,this,_1));
}

QueuedPinger::~QueuedPinger(){}

bool QueuedPinger::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	//Configure desired outputs and rate
	ph.param("enhanced_usbl", enhanced_usbl, enhanced_usbl);
	ph.param("enhanced_data", enhanced_data, enhanced_data);
	ph.param("timeout", timeout, timeout);

	queue_processor = boost::thread(boost::bind(&QueuedPinger::processQueue,this));

	return true;
}


SeatracMessage::Ptr QueuedPinger::makeDataCmd(const underwater_msgs::ModemTransmission::ConstPtr& msg,
		uint8_t msgtype)
{
	DatSendCmd::Ptr cmd(new DatSendCmd());
	cmd->dest = msg->receiver;
	cmd->msg_type = msgtype;
	cmd->data.assign(msg->payload.begin(), msg->payload.end());
	return boost::dynamic_pointer_cast<SeatracMessage>(cmd);
}

SeatracMessage::Ptr QueuedPinger::makePingCmd(const underwater_msgs::ModemTransmission::ConstPtr& msg)
{
	PingSendCmd::Ptr cmd(new PingSendCmd());
	cmd->dest = msg->receiver;
	cmd->msg_type = (enhanced_usbl)?AMsgType::MSG_REQX : AMsgType::MSG_REQU;
	return boost::dynamic_pointer_cast<SeatracMessage>(cmd);
}
SeatracMessage::Ptr QueuedPinger::makeReply(const underwater_msgs::ModemTransmission::ConstPtr& msg)
{
	DatQueueSetCmd::Ptr cmd(new DatQueueSetCmd());
	cmd->dest = msg->receiver;
	cmd->data.assign(msg->payload.begin(), msg->payload.end());
	return boost::dynamic_pointer_cast<SeatracMessage>(cmd);
}

void QueuedPinger::processQueue()
{
	//Wait until something is in the queue or the queue is awoken
	boost::unique_lock<boost::mutex> lock(queue_mux);
	while (outgoing.empty())
	{
		queue_condition.wait(lock));
	}


	SeatracMessage::Ptr message;

	//Debug turn-around time measurement
	static ros::Time ltime;
	ROS_INFO("Turnaround time for USBL messages: %f",(ros::Time::now() - ltime).toSec());
	ltime = ros::Time::now();

	boost::mutex::scoped_lock l(data_mux);
/*	if (outgoing.size())
	{
		message = outgoing.front();
		outgoing.pop();
	}*/
	l.unlock();

	this->sendPkg(message);
}

void QueuedPinger::sendPkg(const SeatracMessage::ConstPtr& message)
{
	if (!(is_busy = this->sender(message)))
	{
		ROS_WARN("QueuedPinger: Message sending failed for CID=0x%d", message->getCid());
		return;
	}

	boost::mutex::scoped_lock ping_lock(ping_mux);
	boost::system_time const maxtime=boost::get_system_time()+boost::posix_time::seconds(timeout);
	while (is_busy)
	{
		if (!ping_condition.timed_wait(ping_lock,maxtime))
		{
			ROS_WARN("QueuedPinger: USBL went into timeout.");
			std_msgs::Bool data;
			data.data = true;
			is_busy = false;
			//Do something on timeout event
			break;
		}
	}
}

void QueuedPinger::unlock()
{
	boost::mutex::scoped_lock lock(ping_mux);
	is_busy = false;
	ping_condition.notify_one();
}

bool QueuedPinger::onPingErrors(const SeatracMessage::ConstPtr& msg)
{
	bool unlock(false);
	uint8_t cid = msg->getCid();

	if (cid == PingError::CID)
	{
		const PingError::ConstPtr err(
				boost::dynamic_pointer_cast<PingError const>(msg));
		ROS_ERROR("QueuedPinger: PingError: 0x%x", err->status);
		unlock = true;
	}
	else if (cid == PingSendResp::CID)
	{
		const PingSendResp::ConstPtr resp(
				boost::dynamic_pointer_cast<PingSendResp const>(msg));
		if ((unlock = (resp->status != CST::OK)))
				ROS_ERROR("QueuedPinger: PingSendResp: 0x%x", resp->status);
	}
	else if (cid == DatSendResp::CID)
	{
		const DatSendResp::ConstPtr resp(
				boost::dynamic_pointer_cast<DatSendResp const>(msg));
		if ((unlock = (resp->status != CST::OK)))
				ROS_ERROR("QueuedPinger: DatSendResp: 0x%x", resp->status);
	}
	else if (cid == DatError::CID)
	{
		const DatError::ConstPtr err(
				boost::dynamic_pointer_cast<DatError const>(msg));
		ROS_ERROR("QueuedPinger: DatError: 0x%x", err->status);
		unlock = true;
	}
	else
	{
		ROS_WARN("QueuedPinger: No handling for PingError: 0x%x",cid);
	}

	if (unlock)
	{
		ROS_DEBUG("QueuedPinger: Unlocking wait.");
		this->unlock();
	}

	return true;
}

bool QueuedPinger::onPingReplies(const SeatracMessage::ConstPtr& msg)
{
	bool unlock(false);
	uint8_t cid = msg->getCid();

	if (cid == PingResp::CID)
	{
		const PingResp::ConstPtr resp(boost::dynamic_pointer_cast<PingResp const>(msg));
		ROS_DEBUG("QueuedPinger: Ping response received from %d.", resp->acofix.src);
	}
	else if (cid == DatReceive::CID)
	{
		const DatReceive::ConstPtr resp(boost::dynamic_pointer_cast<DatReceive const>(msg));
		ROS_DEBUG("QueuedPinger: Data reply received from %d.", resp->acofix.src);
		ROS_INFO("Data size: %d, Data byte 1: %d", resp->data.size(), resp->data[0]);
	}
	else
	{
		ROS_WARN("QueuedPinger: No unhandled CID=%x",cid);
	}

	this->unlock();
	return true;
}

PLUGINLIB_EXPORT_CLASS(labust::seatrac::QueuedPinger, labust::seatrac::DeviceController)
