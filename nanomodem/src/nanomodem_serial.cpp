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
#include <labust/nanomodem/nanomodem_serial.h>
//#include <labust/seatrac/seatrac_factory.h>
//#include <pluginlib/class_list_macros.h>

#include <std_msgs/String.h>

#include <boost/bind.hpp>
//#include <boost/crc.hpp>

using namespace labust::nanomodem;

NanomodemSerial::NanomodemSerial():
		io(),
		port(io){}

NanomodemSerial::~NanomodemSerial()
{
	io.stop();
	runner.join();
}

bool NanomodemSerial::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	std::string port_name("/dev/ttyUSB1");
	int baud(9600);

	ph.param("port_name",port_name, port_name);
	ph.param("baud", baud, baud);

	raw_out = nh.advertise<std_msgs::String>("usbl_raw_msg",1);

	return this->connect(port_name, baud);
}

bool NanomodemSerial::connect(const std::string& port_name, int baud)
try
{
	using namespace boost::asio;
	port.open(port_name);
	port.set_option(serial_port::baud_rate(baud));
	port.set_option(serial_port::flow_control(
			serial_port::flow_control::none));
	port.set_option(serial_port::parity(serial_port::parity::none));

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


void NanomodemSerial::startReceive()
{
	using namespace boost::asio;
	async_read_until(port, buffer, "\r\n", boost::bind(&NanomodemSerial::onData, this, _1,_2));
}

bool NanomodemSerial::send(const std::string& msg)
{
	try
	{
		out = msg;
		//NanomodemFactory::encodePacket(msg, &out);
		ROS_INFO("Sending:%s",out.c_str());
		boost::asio::write(port, boost::asio::buffer(out));
		return true;
	}
	catch (std::exception& e)
	{
		ROS_WARN("%s", e.what());
		return false;
	}
}

bool NanomodemSerial::resend()
{
	try
	{
		ROS_DEBUG("NanomodemSerial: Re-sending data: %s", out.c_str());
		boost::asio::write(port, boost::asio::buffer(out));
		return true;
	}
	catch (std::exception& e)
	{
		std::cerr<<e.what()<<std::endl;
		return false;
	}
}

void NanomodemSerial::onData(const boost::system::error_code& e, std::size_t size)
{
	if (!e)
	{
		std::istream is(&buffer);
		std::string data(size,'\0');
		is.read(&data[0],size);

		ROS_INFO("Received:%s",data.c_str());


		//ROS debug output
		std_msgs::String::Ptr outraw(new std_msgs::String());
		outraw->data = data;
		raw_out.publish(outraw);

		decodeResponse(data);

//		NanomodemMessage::Ptr msg;
//		try
//		{
//			if (NanomodemFactory::decodePacket(data, msg))
//			{
//				boost::mutex::scoped_lock l(callback_mux);
//				if (!callback.empty()) callback(msg);
//			}
//			else
//			{
//				ROS_WARN("NanomodemSerial: Message encoding failed.");
//			}
//		}
//		catch (std::exception& e)
//		{
//			ROS_WARN("NanomodemSerial: %s",e.what());
//		}
	}
	else
	{
		ROS_WARN("NanomodemSerial: Comms reception failed.");
	};

	this->startReceive();

	//this->
}

