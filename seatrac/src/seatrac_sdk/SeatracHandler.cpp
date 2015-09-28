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
#include <labust/seatrac/SeatracHandler.hpp>
#include <boost/bind.hpp>
#include <boost/crc.hpp>
#include <iostream>

using namespace labust::seatrac;

SeaTracHandler::SeaTracHandler():
					io(),
					port(io){}

SeaTracHandler::~SeaTracHandler()
{
	io.stop();
	runner.join();
}

bool SeaTracHandler::connect(const std::string& portName, int baud)
try
{
	using namespace boost::asio;
	port.open(portName);
	port.set_option(serial_port::baud_rate(baud));
	port.set_option(serial_port::flow_control(
			serial_port::flow_control::none));

	bool setupOk = port.is_open();

	if (setupOk)
	{
		//Start the receive cycle
		this->start_receive();
		runner = boost::thread(boost::bind(&boost::asio::io_service::run,&io));
	}

	return setupOk;
}
catch (std::exception& e)
{
	std::cerr<<e.what()<<std::endl;
	return false;
}


void SeaTracHandler::start_receive()
{
	using namespace boost::asio;
	async_read_until(port, buffer,
			"\r\n",
			boost::bind(&SeaTracHandler::onData, this, _1,_2));
}

void SeaTracHandler::convertToBinary(const std::string& data, std::vector<uint8_t>& binary)
{
	binary.resize(data.size()/2,0);
	for(int i=0; i<binary.size(); ++i)
	{
		std::stringstream d2;
		d2<<data[2*i]<<data[2*i+1];
		int temp;
		d2>>std::hex>>temp;
		binary[i] = temp;
	}
}

void SeaTracHandler::convertToAscii(const std::vector<uint8_t>& binary, std::string& data)
{
	std::stringstream out;
	for(int i=0; i<binary.size(); ++i)
	{
		out.width(2);
		out.fill('0');
		out<<std::hex<<std::fixed<<int(binary[i]);
	}

	data = out.str();
}

bool SeaTracHandler::send(int cid, const std::vector<uint8_t>& binary)
try
{
	std::vector<uint8_t> data(binary);
	data.insert(data.begin(),cid);
	boost::crc_16_type checksum;
	checksum.process_bytes(&data[0], data.size());
	uint16_t chk = checksum.checksum();
	uint8_t* pt = reinterpret_cast<uint8_t*>(&chk);
	data.push_back(pt[0]);
	data.push_back(pt[1]);

	convertToAscii(data, out);
	out.insert(out.begin(), '#');
	out.push_back('\r');
	out.push_back('\n');
	std::cout<<"Sending data:"<<out<<std::endl;
	boost::asio::write(port, boost::asio::buffer(out));
	return true;
}
catch (std::exception& e)
{
	std::cerr<<e.what()<<std::endl;
	return false;
}

bool SeaTracHandler::resend()
try
{
	std::cout<<"Re-sending data:"<<out<<std::endl;
	boost::asio::write(port, boost::asio::buffer(out));
}
catch (std::exception& e)
{
	std::cerr<<e.what()<<std::endl;
	return false;
}

void SeaTracHandler::onData(const boost::system::error_code& e,
		std::size_t size)
{
	if (!e)
	{
		std::istream is(&buffer);
		std::string data(size,'\0');
		is.read(&data[0],size);

		std::cout<<"Received data:"<<data;

		if (data[0] == '$')
		{
			//Skip '$' and '\r\n'
			std::vector<uint8_t> binary;
			convertToBinary(data.substr(dataStart, data.size()-dataTrunc), binary);
			int cid = binary[0];
			//Checksum
			boost::crc_16_type checksum;
			checksum.process_bytes(binary.data(), binary.size()-2);
			uint16_t* chk = reinterpret_cast<uint16_t*>(&binary[binary.size()-2]);

			if ((*chk) == checksum.checksum())
			{
				std::cout<<"Response received. Checksum ok."<<std::endl;
				binary.erase(binary.begin());
				binary.pop_back(); binary.pop_back();
				if (!callback.empty()) callback(cid, binary);
			}
			else
			{
				std::cerr<<"Checksum failed."<<std::endl;
			}
		}
	}

	this->start_receive();
}


