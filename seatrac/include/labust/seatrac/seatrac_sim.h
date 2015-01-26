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
#ifndef SEATRAC_SEATRACSIM_H
#define SEATRAC_SEATRACSIM_H
#include <labust/seatrac/seatrac_comms.h>

#include <auv_msgs/NavSts.h>
#include <underwater_msgs/MediumTransmission.h>
#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

namespace labust
{
	namespace seatrac
	{
		/**
		 * The class implements the Seatrac ROS simulation.
		 */
		class SeatracSim : public virtual SeatracComms
		{
			enum
			{
				IDLE=0,
				WAIT_PING_REPLY,
				WAIT_DATA_REPLY
			};
		public:
			/**
			 * Main constructor
			 */
			SeatracSim();
			///Default destructor
			~SeatracSim();
			/**
			 * Configures the serial port based on the ROS node handle.
			 */
			bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph);

			/**
			 * Register the message handler.
			 */
			virtual void registerCallback(const CallbackType& callback)
			{
				boost::mutex::scoped_lock l(callback_mux);
				this->callback = callback;
			};
			/**
			 * Message send
			 */
			virtual bool send(const SeatracMessage::ConstPtr& msg);

			///Resend last message
			virtual bool resend();

		private:
			///Navsts handler
			void onNavSts(const auv_msgs::NavSts::ConstPtr& msg);
			///Medium transmission handler
			void onMediumTransmission(const
				underwater_msgs::MediumTransmission::ConstPtr& msg);
			///Helper method for received Pings
			void processPingCmd(const
				underwater_msgs::MediumTransmission::ConstPtr& msg);
			///Helper method for received Data
			void processDataCmd(const
							underwater_msgs::MediumTransmission::ConstPtr& msg);
			///Helper method for USBL loop
			void onUSBLTimeout(const ros::TimerEvent& e);
			///Helper method for registration
			void registerModem();

			///Helper function for medium message sending
			inline void sendToMedium(underwater_msgs::MediumTransmission::Ptr& msg)
			{
				//Attach current state to message
				boost::mutex::scoped_lock l(position_mux);
				msg->position = navstate;
				l.unlock();
				//Publish message
				medium_out.publish(msg);
			}

			inline void sendMessage(const SeatracMessage::ConstPtr& msg)
			{
				boost::mutex::scoped_lock l(callback_mux);
				if (callback) callback(msg);
			}

			inline void startTimer(double wait_time)
			{
				sleeper.setPeriod(ros::Duration(wait_time));
				sleeper.start();
			}

			inline void fillAcoFix(AcoFix& acfix)
			{
				acfix.vos = vos;
				boost::mutex::scoped_lock l(position_mux);
				acfix.attitude[Status::ROLL] = navstate.orientation.roll*Status::ATT_SC;
				acfix.attitude[Status::PITCH] = navstate.orientation.pitch*Status::ATT_SC;
				acfix.attitude[Status::YAW] = navstate.orientation.yaw*Status::ATT_SC;
				acfix.depth_local = navstate.position.depth;
			}

			///Helper method to make state behave as atomic
			inline int getState()
			{
				boost::mutex::scoped_lock ls(state_mux);
				return state;
			}
			///Helper method to set state
			inline void setState(int state)
			{
				boost::mutex::scoped_lock ls(state_mux);
				this->state = state;
			}

			template <class Type>
			inline void sendError(int error)
			{
				typename Type::Ptr err(new Type());
				err->status = error;
				err->beacon_id = node_id;
				this->sendMessage(boost::dynamic_pointer_cast<SeatracMessage const>(err));
			}

			///Device position subscriber
			ros::Subscriber navsts;
			///Incoming data from medium subscriber
			ros::Subscriber medium_in;
			///Outgoing data to medium
			ros::Publisher medium_out;
			///Muxer for the publisher
			boost::mutex medium_mux;

			///The message callback
			CallbackType callback;
			///Muxer for the callback
			boost::mutex callback_mux;
			///Muxer for the node position
			boost::mutex position_mux;
			///Internal state muxer
			boost::mutex state_mux;
			///USBL timeout thread
			ros::Timer sleeper;

			///The internal simulator state (default: IDLE)
			int state;
			///Expected reply source id (default: 0)
			int expected_id;
			///The simulated node ID (default: 1)
			int node_id;
			///The simulated ping duration (default: 0.65)
			double ping_duration;
			///The maximum distance (default: 500)
			double max_distance;
			///The speed of sound (default: 1500)
			double vos;
			///The simulation timeout overhead (default: 0.1)
			double time_overhead;
			///The node position and attitude
			auv_msgs::NavSts navstate;
			///Modem registration flag
			bool registered;
		};
	}
}

/* SEATRAC_SEATRACSIM_H */
#endif
