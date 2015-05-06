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
#ifndef NOVATEL_NOVATEL_NODE_H
#define NOVATEL_NOVATEL_NODE_H
#include <labust/navigation/novatel_messages.h>
#include <labust/comms/ascii_serial.h>

#include <ros/ros.h>

#include <string>

namespace labust
{
	namespace navigation
	{
		/**
		 * The class implements a simple BESTPOS and BESTVEL novatel reader.
		 */
		class NovatelNode
		{
		public:
			///Main constructor
			NovatelNode();
			///Generic destructor.
			~NovatelNode();

			///Initialize the node
			void onInit();

		private:
			///Handle incoming messages.
			void onData(const std::string& data);
			///Helper method for header comparison
			bool testHeader(const std::string& match, const std::string& stream);
			///BESTPOSA message handler
			void onBestPos(BestPos& pos);
			///BESTVELA message handler
			void onBestVel(BestVel& vel);

			///Position set subsciber
			ros::Subscriber posset_sub;

			///The GPS fix publisher
			ros::Publisher fix_pub;
			///The GPS velocity publisher
			ros::Publisher vel_pub;

			///The ASCII serial port
			labust::comms::ASCIISerial serial;
		};
	}
}

/* NOVATEL_NOVATEL_NODE_H */
#endif
