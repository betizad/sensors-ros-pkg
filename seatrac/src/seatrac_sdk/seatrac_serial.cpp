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
#include <labust/seatrac/seatrac_serial.h>
#include <labust/seatrac/seatrac_factory.h>
#include <pluginlib/class_list_macros.h>

#include <boost/bind.hpp>
#include <boost/crc.hpp>

using namespace labust::seatrac;

SeatracSerial::SeatracSerial():
		io(),
		port(io){}

SeatracSerial::~SeatracSerial()
{
	io.stop();
	runner.join();
}

bool SeatracSerial::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	std::string port_name("/dev/ttyS0");
	int baud(115200);

	ph.param("port_name",port_name, port_name);
	ph.param("baud", baud, baud);

	return this->connect(port_name, baud);
}

bool SeatracSerial::connect(const std::string& port_name, int baud)
try
{
	using namespace boost::asio;
	port.open(port_name);
	port.set_option(serial_port::baud_rate(baud));
	port.set_option(serial_port::flow_control(
			serial_port::flow_control::none));

	bool setupOk = port.is_open();

	if (setupOk)
	{
		//Start the receive cycle
		this->startReceive();
		runner = boost::thread(boost::bind(&boost::asio::io_service::run,&io));
	}

	return setupOk;
}
catch (std::exception& e)
{
	std::cerr<<e.what()<<std::endl;
	return false;
}


void SeatracSerial::startReceive()
{
	using namespace boost::asio;
	async_read_until(port, buffer,
			"\r\n",
			boost::bind(&SeatracSerial::onData, this, _1,_2));
}

bool SeatracSerial::send(const SeatracMessage::ConstPtr& msg)
try
{
	SeatracFactory::encodePacket(msg, &out);
	ROS_INFO("Sending:%s",out.c_str());
	boost::asio::write(port, boost::asio::buffer(out));
	return true;
}
catch (std::exception& e)
{
	ROS_ERROR("%s", e.what());
	return false;
}

bool SeatracSerial::resend()
try
{
	ROS_DEBUG("SeatracSerial: Re-sending data: %s", out.c_str());
	boost::asio::write(port, boost::asio::buffer(out));
	return true;
}
catch (std::exception& e)
{
	std::cerr<<e.what()<<std::endl;
	return false;
}

void SeatracSerial::onData(const boost::system::error_code& e,
		std::size_t size)
{
	if (!e)
	{
		std::istream is(&buffer);
		std::string data(size,'\0');
		is.read(&data[0],size);

		SeatracMessage::Ptr msg;
		try
		{
			if (SeatracFactory::decodePacket(&data, msg))
			{
				boost::mutex::scoped_lock l(callback_mux);
				if (!callback.empty()) callback(msg);
			}
			else
			{
				ROS_WARN("SeatracSerial: Message encoding failed.");
			}
		}
		catch (std::exception& e)
		{
			ROS_ERROR("SeatracSerial: %s",e.what());
		}
	}
	else
	{
		ROS_WARN("SeatracSerial: Comms reception failed.");
	};

	this->startReceive();
}

PLUGINLIB_EXPORT_CLASS(labust::seatrac::SeatracSerial, labust::seatrac::SeatracComms)

