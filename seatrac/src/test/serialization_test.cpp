/*
 * serialization_test.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: dnad
 */
#include <labust/archive/asciihex_iarchive.hpp>
#include <labust/seatrac/SeatracHandler.hpp>
#include <labust/seatrac/SeatracMessages.hpp>
#include <labust/preprocessor/mem_serialized_struct.hpp>
#include <labust/preprocessor/clean_serializator.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>

#include <cstdint>
#include <string>
#include <sstream>
#include <ios>
#include <vector>

#include <ros/ros.h>

#include <boost/thread.hpp>
#include <labust/seatrac/SeatracCID.hpp>
#include <labust/seatrac/SeatracDefinitions.hpp>

PP_LABUST_CLEAN_ARRAY_ISERIALIZATOR_IMPL(boost::archive::binary_iarchive)
PP_LABUST_CLEAN_ARRAY_OSERIALIZATOR_IMPL(boost::archive::binary_oarchive)

using namespace labust::seatrac;

bool isUSBL = true;
SeaTracHandler comms;
bool usblBusy = false;
boost::mutex pingLock;
double ping_timeout = 10;
boost::condition_variable usblCondition;

bool msgReq = false;

ros::Time* lastPing;

std::string ex_data()
{
	return "$0282330000011B0301690E000000000000FF900301006901B7FAC5BFFF910301007A07750463A95DDE";
}

void process(int cid, std::vector<uint8_t>& data)
{
	std::cout<<"Received: cid="<<cid<<std::endl;
	using namespace labust::seatrac;

	if (cid == CID_XCVR::fix)
	{
		ROS_INFO("Fix:");
		std::istringstream in;
		in.rdbuf()->pubsetbuf(reinterpret_cast<char*>(data.data()), data.size());
		boost::archive::binary_iarchive inSer(in, boost::archive::no_header);
		XcvrFix rec;
		inSer >> rec;
		ROS_INFO("\t Beacon ID:%d", rec.beaconId);
		ROS_INFO("\t Signal valid:%d", rec.signal_valid);
		ROS_INFO("\t Depth valid:%d", rec.depth_valid);
		ROS_INFO("\t Range valid:%d", rec.range_valid);
		ROS_INFO("\t Range:%d", rec.range_dist);
		ROS_INFO("\t Remote depth:%d", rec.depth_remote);
		ROS_INFO("\t East:%d, North:%d, Depth:%d", rec.position[0], rec.position[1], rec.position[2]);
		ROS_INFO("\t Remaining:%d", in.rdbuf()->in_avail());
	}

	if (isUSBL)
	{
		if ((cid == CID_PING::error) || (cid == CID_DATA::dat_error) || (cid == CID_DATA::receive))
		{
			{
				boost::mutex::scoped_lock lock(pingLock);
				usblBusy = false;
			}
			usblCondition.notify_one();
			ROS_INFO("Turnaround: %f",(ros::Time::now()-*lastPing).toSec());
		}

		if (cid == CID_DATA::receive)
		{
			ROS_INFO("Received:");
			std::istringstream in;
			in.rdbuf()->pubsetbuf(reinterpret_cast<char*>(data.data()), data.size());
			boost::archive::binary_iarchive inSer(in, boost::archive::no_header);
			DatReceive rec;
			inSer >> rec;
			for (int i=0; i< rec.payload.size(); ++i) 	ROS_INFO("\t %d", rec.payload[i]);
		}
	}
	else
	{
		//We need to track that the acknowledge was sent back
		if (cid == CID_XCVR::rx_req) msgReq = true;
		if (cid == CID_DATA::send)
		{
			msgReq = !(data[0] == 0);
		}

		if (msgReq)
		{
			msgReq = false;
			std::cout<<"Send reply"<<std::endl;
			//Send back datagram
			std::vector<uint8_t> data;
			DatSend msg;
			msg.destId = 1;
			msg.flags = DatMode::no_ack;
			msg.payload.push_back(48);
			msg.payload.push_back(49);
			msg.payload.push_back(50);
			msg.payload.push_back(51);
			msg.payload.push_back(52);
			msg.payload.push_back(53);

			std::ostringstream out;
			boost::archive::binary_oarchive outSer(out, boost::archive::no_header);
			outSer << msg;
			std::string result = out.str();
			data.insert(data.begin(), result.begin(), result.end());
			comms.send(CID_DATA::send, data);
		}

	}
}

void sendUSBLPkg()
{
	//Send ping
	std::vector<uint8_t> data;
	//data.push_back(2);

	// Send datagram
	DatSend msg;
	msg.destId = 2;
	msg.flags = DatMode::ack_usbl;
	msg.payload.push_back(48);
	msg.payload.push_back(49);

	msg.payload.push_back(50);
	msg.payload.push_back(51);
	msg.payload.push_back(52);
	msg.payload.push_back(53);
	msg.payload.push_back(54);

	std::ostringstream out;
	boost::archive::binary_oarchive outSer(out, boost::archive::no_header);
	outSer << msg;
	std::string result = out.str();
	data.insert(data.begin(), result.begin(), result.end());

	*lastPing = ros::Time::now();
	comms.send(CID_DATA::send, data);

	usblBusy = true;

	boost::mutex::scoped_lock lock(pingLock);
	boost::system_time const timeout=boost::get_system_time()+boost::posix_time::seconds(ping_timeout);
	while (usblBusy)
	{
		if (!usblCondition.timed_wait(lock,timeout))
		{
			ROS_INFO("USBL went into timeout.");
			break;
		}
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "tester");
	ros::Time::init();
	lastPing = new ros::Time();

	if (argc > 1)
	{
		comms.connect(argv[1],115200);
	}
	else
	{
		comms.connect("/dev/ttyUSB1",115200);
	}
	comms.registerCallback(process);

	if (argc > 2) isUSBL = (argv[2] == "0");

	ros::Rate rate(10);

	while (ros::ok())
	{
		if (isUSBL)	sendUSBLPkg();
		else
			rate.sleep();
	};

	delete lastPing;

  return 0;
}
