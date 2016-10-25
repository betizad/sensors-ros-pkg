/*
 * nanomodem_protocol.h
 *
 *  Created on: Jul 12, 2016
 *      Author: filip
 */

#ifndef SENSORS_ROS_PKG_NANOMODEM_INCLUDE_LABUST_NANOMODEM_NANOMODEM_PROTOCOL_H_
#define SENSORS_ROS_PKG_NANOMODEM_INCLUDE_LABUST_NANOMODEM_NANOMODEM_PROTOCOL_H_

#include <string>
#include <iostream>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
/*
Serial communication format is (9600, 8, n, 1) with no flow control.
All commands issued to the Nanomodem are prefixed with ‘$’.
All responses from the Nanomodem are prefixed with ‘#’ and terminated with <CR><LF>.
Unrecognised or invalid commands return ‘E’ to indicate an error.

$Axxx - set node address to xxx (ascii decimal e.g. 123)
#Axxx - confirms node address has been set to xxx

$? - Query Nanomodem status
#AxxxVyyyy - where xxx is node address and yyyy is 10-bit battery voltage monitor value.
             To convertto a voltage: v = yyyy * 1.1*6/1024

*/
//TODO Complete protocol description

namespace labust
{
	namespace nanomodem
	{
		class NanomodemProtocol
		{
		public:

			NanomodemProtocol():waiting_confirmation_flag(false)
			{

			}

			~NanomodemProtocol()
			{

			}

			inline std::string setNodeAddress(const int& node_address)
			{
				return commandFormat("A" + boost::str(boost::format("%03d") % node_address));
			}

			inline std::string queryNodeStatus()
			{
				return commandFormat("?");
			}

			inline std::string pingNode(const int& node_address)
			{
				return commandFormat("P" + boost::str(boost::format("%03d") % node_address));
			}

			inline std::string queryBatteryVoltage(const int& node_address)
			{
				return commandFormat("V" + boost::str(boost::format("%03d") % node_address));
			}
//TODO check for data length
			std::string unicastDataMessage(const std::string& data, const int& node_address)
			{
				int data_length = data.length();
				return commandFormat("U" +
						boost::str(boost::format("%03d") % node_address) +
						boost::str(boost::format("%d") % data_length) +
						data);
			}

			std::string broadcastDataMessage(const std::string& data)
			{
				int data_length = data.length();
				return commandFormat("B" +
						boost::str(boost::format("%d") % data_length) +
						data);
			}

		private:

			inline std::string commandFormat(const std::string& str){
				return "$" + str;

			}

		protected:

			void decodeResponse(const std::string& data)
			{
				char prefix(data[0]);
				if(prefix == '#')
				{
					char response_id(data[1]);
					/*** Erase <CR><LF> ***/
					std::string parsed(data,2, (data.length()-4));

					switch(response_id)
					{
						case 'A':
						{
							//ROS_ERROR("data length %d",data.length());
							//ROS_ERROR("length %d",parsed.length());
							if(parsed.length() == 3)
							{
								std::cout << "New node address: " << parsed << std::endl;
							}
							else if(parsed.length() == 8)
							{
								std::cout << "Node address: " << parsed.substr(0,3) << std::endl;
								std::cout << "Battery voltage: " << parsed.substr(4,4) << std::endl;
							}
							break;
						}
						case 'P':
						{
							if(parsed.length() == 3)
							{
								std::cout << "Ping address: " << parsed << std::endl;
							}
							break;
						}
						case 'R':
						{
							if(parsed.length() == 9)
							{
								std::cout << "Response from node address: " << parsed.substr(0,3) << std::endl;
								std::cout << "Range: " << parsed.substr(4,5) << std::endl;
							} else if(parsed.length() == 8)
							{
								std::cout << "Response from node address: " << parsed.substr(0,3) << std::endl;
								std::cout << "Battery voltage: " << parsed.substr(4,4) << std::endl;
							}
							break;
						}
						case 'V':
						{
							if(parsed.length() == 3)
							{
								std::cout << "Requested battery voltage on address: " << parsed << std::endl;
							}
							break;
						}
						case 'U':
						{
							if(parsed.length() == 4 && waiting_confirmation_flag)
							{
								waiting_confirmation_flag = false;
								std::cout << "Sent " << parsed.substr(3,1) << "bytes to node address: " << parsed.substr(0,3) << std::endl;
							}
							else if(parsed.length() <= 8)
							{
								std::cout << "Received " << parsed.substr(0,1) << " bytes from node address: NOT KNOWN" << std::endl;
								std::cout << "Received  data: "  << parsed.substr(1,boost::lexical_cast<int>(parsed.substr(0,1))) << std::endl;

							}
							break;
						}
						case 'B':
						{
							if(parsed.length() == 1 && waiting_confirmation_flag)
							{
								waiting_confirmation_flag = false;
								std::cout << "Broadcasted " << parsed.substr(0,1) << " bytes." << std::endl;
							}
							else if(parsed.length() <= 8)
							{
								std::cout << "Received broadcasted " << parsed.substr(3,1) << " bytes from node address: " << parsed.substr(0,3) << std::endl;
								std::cout << "Received  data: "  << parsed.substr(1,boost::lexical_cast<int>(parsed.substr(0,1))) << std::endl;

							}
							break;
						}
						default:
						{

						}
					}
				}
				else if(prefix == 'E')
				{
					std::cout << "Error" << std::endl;
				}
				else
				{

				}

			}

			inline void setWaitingConfirmationFlag(const bool value)
			{
				waiting_confirmation_flag = value;
			}

			bool waiting_confirmation_flag;
		};
	}
}



#endif /* SENSORS_ROS_PKG_NANOMODEM_INCLUDE_LABUST_NANOMODEM_NANOMODEM_PROTOCOL_H_ */
