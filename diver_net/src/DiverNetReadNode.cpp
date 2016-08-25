#include <labust/sensors/DiverNetReadNode.h>
#include <labust/tools/conversions.hpp>

#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <Eigen/Dense>

#include <boost/bind.hpp>
#include <boost/crc.hpp>

#include <iosfwd>

using namespace labust::sensors;

DiverNetReadNode::DiverNetReadNode():
							io(),
							port(io),
							ringBuffer(headerSize,0),
							nodeCount(20) {
	this->onInit();
}

DiverNetReadNode::~DiverNetReadNode() {
	io.stop();
	runner.join();
}

void DiverNetReadNode::onInit() {
	ros::NodeHandle nh, ph("~");
	ros::Rate r(1);
	bool setupOk(false);
	while (!(setupOk = this->setup_port()) && ros::ok()) {
		ROS_ERROR("DiverNetNode::Failed to open port.");
		r.sleep();
	}

	//Setup publisher
	rawData = nh.advertise<std_msgs::Int16MultiArray>("net_data",1);
  temperature_pub = nh.advertise<std_msgs::Float32>("temperature",1);
  pressure_pub = nh.advertise<std_msgs::Float32>("pressure",1);
  hr_pulse_pub = nh.advertise<std_msgs::Bool>("hr_pulse",1);
	netInit = nh.subscribe<std_msgs::Bool>("net_init",1,&DiverNetReadNode::onNetInit, this);

	if (setupOk) {
		ROS_INFO("DiverNet is connected.");
		//Configure the device
		this->configureNet();
		//Start the receive cycle
		this->start_receive();
		runner = boost::thread(boost::bind(&boost::asio::io_service::run,&io));
	}
}

void DiverNetReadNode::configureNet() {
	//Read number of nodes
	ros::NodeHandle ph("~");
	ph.param("node_count", nodeCount, nodeCount);
	rawBuffer.resize(nodeCount * dataPerNode + crc + pressureData + temperatureData + hrPulseData);
}

void DiverNetReadNode::start_receive() {
	using namespace boost::asio;
	async_read(port, buffer.prepare(headerSize),
			boost::bind(&DiverNetReadNode::onHeader, this, _1,_2));
}

bool DiverNetReadNode::setup_port() {
	ros::NodeHandle ph("~");
	std::string portName("/dev/ttyUSB0");
	int baud(230400);

	ph.param("PortName",portName,portName);
	ph.param("Baud",baud,baud);

	using namespace boost::asio;
	port.open(portName);
	struct termios cfg;
	tcgetattr(port.native(), &cfg);
	cfsetspeed(&cfg, baud);
	tcsetattr(port.native(), TCSANOW, &cfg);

	//port.set_option(serial_port::baud_rate(baud));
	port.set_option(serial_port::flow_control(serial_port::flow_control::none));
	port.set_option(serial_port::parity(serial_port::parity::none));
	port.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
	

	return port.is_open();
}

void DiverNetReadNode::onHeader(const boost::system::error_code& e,
		std::size_t size) {
	if (!e) {
		buffer.commit(size);
		if (size == 1) {
			//Put the new byte on the end of the ring buffer
			ringBuffer.push_back(buffer.sbumpc());
		} else {
			//Copy all data into the buffer
			buffer.sgetn(reinterpret_cast<char*>(ringBuffer.data()),size);
		}

		//Check LRC
		if (test_sync()) {
			std::cout<<"Sync ok."<<std::endl;
			boost::asio::async_read(port, boost::asio::buffer(rawBuffer),
					boost::bind(&DiverNetReadNode::onData,this,_1,_2));
			return;
		} else {
			std::cerr<<"Sync failed."<<std::endl;
			ringBuffer.erase(ringBuffer.begin());
			boost::asio::async_read(port,
					buffer.prepare(1),
					boost::bind(&DiverNetReadNode::onHeader,this,_1,_2));
			return;
		}
	} else {
		ROS_ERROR("DiverNetNode: %s",e.message().c_str());
	}
	this->start_receive();
}

bool DiverNetReadNode::test_sync() {
	std::string header(reinterpret_cast<char*>(ringBuffer.data()),headerSize);

	std::cout<<"SyncData:"<<header<<std::endl;

	return (header == "D1Ve");
}

void DiverNetReadNode::onData(const boost::system::error_code& e,
		std::size_t size) {
  if (!e) {
		boost::crc_16_type result;
		uint16_t crc_test = 256*rawBuffer[rawBuffer.size()-1]+rawBuffer[rawBuffer.size()-2];

		result.process_bytes(rawBuffer.data(),rawBuffer.size()-2);
    // std::cerr << crc_test << " " << result.checksum() << std::endl;
		if ((result.checksum() == crc_test)) {
			//Process tested data
			//This can be done better
			std::cout << "CRC ok."<<std::endl;
			std_msgs::Int16MultiArrayPtr out(new std_msgs::Int16MultiArray());
			out->data.resize((nodeCount * dataPerNode)/2);

			int elemCount = 9;
			for (int i=0; i<nodeCount; ++i) {
				for (int e=0; e<elemCount; ++e) {
					out->data[i*elemCount + e] = rawBuffer[2*e*nodeCount + i] +
							256*rawBuffer[(2*e+1)*nodeCount + i];
				}
			}
      std_msgs::Float32Ptr pressure_ptr(new std_msgs::Float32());
      std_msgs::Float32Ptr temperature_ptr(new std_msgs::Float32());
      std_msgs::BoolPtr hr_pulse_ptr(new std_msgs::Bool());
      float pressure, temperature;
      memcpy(&pressure_ptr->data, &rawBuffer[nodeCount*dataPerNode], sizeof(pressure_ptr->data));
      memcpy(&temperature_ptr->data, &rawBuffer[nodeCount*dataPerNode+pressureData], sizeof(temperature_ptr->data));
      hr_pulse_ptr->data = rawBuffer[nodeCount*dataPerNode+pressureData+temperatureData] != 0;
      
      pressure_pub.publish(pressure_ptr);    
      temperature_pub.publish(temperature_ptr);
      hr_pulse_pub.publish(hr_pulse_ptr);
      rawData.publish(out);
		} else {
			std::cerr<<"Data CRC failed."<<std::endl;
		}
	}
	this->start_receive();
}

void DiverNetReadNode::onNetInit(const std_msgs::Bool::ConstPtr& init) {
  if (init->data) {
    boost::mutex::scoped_lock l(dataMux);
  }
}


int main(int argc, char* argv[]) {
	ros::init(argc, argv, "diver_net_read_node");
	DiverNetReadNode node;
	ros::spin();
	return 0;
}
