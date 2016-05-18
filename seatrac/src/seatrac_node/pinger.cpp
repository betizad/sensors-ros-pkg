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
#include <labust/seatrac/pinger.h>
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/seatrac_definitions.h>
#include <labust/seatrac/mediator.h>

#include <string>
#include <ros/ros.h>

using namespace labust::seatrac;

Pinger::Pinger(SeatracComms::CallbackType& sender, MessageListener::RegisterMap& registrations):
	sender(sender),
	is_busy(false)
{
	registrations[PingError::CID].push_back(boost::bind(&Pinger::onPingErrors,this,_1));
	registrations[PingSendResp::CID].push_back(boost::bind(&Pinger::onPingErrors,this,_1));
	registrations[PingResp::CID].push_back(boost::bind(&Pinger::onPingReplies,this,_1));
	registrations[DatError::CID].push_back(boost::bind(&Pinger::onPingErrors,this,_1));
	registrations[DatSendResp::CID].push_back(boost::bind(&Pinger::onPingErrors,this,_1));
	registrations[DatReceive::CID].push_back(boost::bind(&Pinger::onPingReplies,this,_1));
}

Pinger::~Pinger()
{
	this->unlock();
}

bool Pinger::send(const SeatracMessage::ConstPtr& msg, double timeout, bool wait_for_ack)
{
	if (msg == 0) return false;
	if (sender.empty()) return false;

	if (!(is_busy = this->sender(msg)))
	{
		ROS_WARN("Pinger: Message sending failed for CID=0x%d", msg->getCid());
		return false;
	}

	if (wait_for_ack)
	{
		boost::mutex::scoped_lock ping_lock(ping_mux);
		boost::system_time const maxtime=boost::get_system_time()+boost::posix_time::seconds(timeout);
		while (is_busy)
		{
			if (!ping_condition.timed_wait(ping_lock,maxtime))
			{
				ROS_WARN("Pinger: USBL went into timeout.");
				is_busy = false;
				return false;
			}
		}
	}

	return !is_error;
}

void Pinger::unlock()
{
	boost::mutex::scoped_lock lock(ping_mux);
	is_busy = false;
	ping_condition.notify_one();
}

bool Pinger::onPingErrors(const SeatracMessage::ConstPtr& msg)
{
	bool unlock(false);
	uint8_t cid = msg->getCid();

	is_error = true;

	if (cid == PingError::CID)
	{
		const PingError::ConstPtr err(
				boost::dynamic_pointer_cast<PingError const>(msg));
		ROS_WARN("Pinger: PingError: 0x%x", err->status);
		unlock = true;
	}
	else if (cid == PingSendResp::CID)
	{
		const PingSendResp::ConstPtr resp(
				boost::dynamic_pointer_cast<PingSendResp const>(msg));
		if ((unlock = (resp->status != CST::OK)))
			ROS_WARN("Pinger: PingSendResp: 0x%x", resp->status);
	}
	else if (cid == DatSendResp::CID)
	{
		const DatSendResp::ConstPtr resp(
				boost::dynamic_pointer_cast<DatSendResp const>(msg));
		if ((unlock = (resp->status != CST::OK)))
			ROS_WARN("Pinger: DatSendResp: 0x%x", resp->status);
	}
	else if (cid == DatError::CID)
	{
		const DatError::ConstPtr err(
				boost::dynamic_pointer_cast<DatError const>(msg));
		ROS_WARN("Pinger: DatError: 0x%x", err->status);
		unlock = true;
	}
	else
	{
		ROS_WARN("Pinger: No handling for PingError: 0x%x",cid);
	}

	if (unlock)
	{
		ROS_DEBUG("Pinger: Unlocking wait.");
		this->unlock();
	}

	return true;
}

bool Pinger::onPingReplies(const SeatracMessage::ConstPtr& msg)
{
	bool unlock(false);
	uint8_t cid = msg->getCid();

	is_error = false;

	if (cid == PingResp::CID)
	{
		const PingResp::ConstPtr resp(boost::dynamic_pointer_cast<PingResp const>(msg));
		ROS_DEBUG("Pinger: Ping response received from %d.", resp->acofix.src);
	}
	else if (cid == DatReceive::CID)
	{
		const DatReceive::ConstPtr resp(boost::dynamic_pointer_cast<DatReceive const>(msg));
		ROS_DEBUG("Pinger: Data reply received from %d.", resp->acofix.src);
		if (resp->data.size())
		{
			ROS_DEBUG("Data size: %d, Data[0]: %d", resp->data.size(), resp->data[0]);
		}
		else
		{
			ROS_INFO("Empty message received.");
		}
	}
	else
	{
		ROS_WARN("Pinger: No unhandled CID=%x",cid);
	}

	this->unlock();
	return true;
}
