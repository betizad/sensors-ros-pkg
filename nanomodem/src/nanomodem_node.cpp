/*
 * nanomodem_node.cpp
 *
 *  Created on: Jul 12, 2016
 *      Author: filip
 */


#include <labust/nanomodem/nanomodem_node.h>

#include <ros/ros.h>

/*********************************************************************
 ***  Main function
 ********************************************************************/

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nanomodem_node");
	ros::NodeHandle nh,ph("~");

	labust::nanomodem::NanomodemSerial NS;
	NS.configure(nh,ph);

	labust::nanomodem::NanomodemProtocol NP;


	/*** Time sample (in seconds) ***/
	double Ts(5.0);
	ros::Rate rate(1/Ts);

	while (ros::ok())
	{

		NS.send(NS.queryNodeStatus());
		//NS.send(NP.queryBatteryVoltage(0));


		ros::spinOnce();
		rate.sleep();
	}



	ros::spin();
	return 0;
}
