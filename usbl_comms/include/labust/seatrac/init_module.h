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
#ifndef USBL_COMMS_INIT_MODULE_H
#define USBL_COMMS_INIT_MODULE_H
#include <labust/comms/caddy/caddy_messages.h>
#include <labust/tools/conversions.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>

#include <boost/thread.hpp>
#include <Eigen/Dense>

namespace labust
{
	namespace seatrac
	{
		/**
		 * The class implements the initialization handling.
		 */
		class InitModule
		{
			typedef message_filters::TimeSynchronizer<geometry_msgs::PointStamped, geometry_msgs::PointStamped> TimeSync;
			typedef boost::shared_ptr< TimeSync > TimeSyncPtr;
			typedef message_filters::Subscriber<geometry_msgs::PointStamped> FilterSub;
			typedef boost::shared_ptr< FilterSub > FilterSubPtr;

		public:
			///Main constructor
			InitModule():
			  init_update(true),
			  allow_external(false),
			  init_offset(Eigen::Vector3d::Zero()),
			  init_llh(Eigen::Vector3d::Zero())
			  {}
			///Default destructor
			~InitModule(){};

			///Listener configuration.
			bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
			{
			  reinitned_sub.reset(new FilterSub(nh, "acoustic_origin_ned", 1));
			  reinitllh_sub.reset(new FilterSub(nh, "acoustic_origin_llh", 1));
			  sync.reset(new TimeSync(*reinitned_sub, *reinitllh_sub, 1));
			  sync->registerCallback(&InitModule::onReinit, this);
			}

			///Return the latitude and longitude of the init position.
			const Eigen::Vector3d& llh(){return init_llh;}
            ///Return the ned offset of the init position.
            const Eigen::Vector3d& offset(){return init_offset;}
			///Return if an init update occured
			bool isNewInit()
			{
			  bool retVal = init_update;
			  init_update = false;
			  return retVal;
			}

			///Process the given payload data
	        template <class ReportMessage>
	        void updateInit(const ReportMessage& data)
	        {
	          // Check if initialization in progress
	          if (data.inited == 0)
	          {
	            //Do something with origin_lon, origin_lat
	          }
	        }

		private:
			/// Handles reinitialization requests
			void onReinit(const geometry_msgs::PointStamped::ConstPtr& ned,
			    const geometry_msgs::PointStamped::ConstPtr& llh)
			{
			  if (allow_external)
			  {
			    labust::tools::pointToVector(ned->point, init_offset);
			    labust::tools::pointToVector(llh->point, init_llh);
			    init_update = true;
			  }
			}

			/// Re-init subcriber for NED coordinates
			FilterSubPtr reinitned_sub;
			/// Re-init subscriber for LLH coordinates
			FilterSubPtr reinitllh_sub;
			/// Message synchronizer
			TimeSyncPtr sync;

			/// Initialization offset
			Eigen::Vector3d init_offset;
			/// The llh position if the init
			Eigen::Vector3d init_llh;
			/// Flag to indicate a new init update.
			bool init_update;
			/// Allow external init updates.
			bool allow_external;
		};
	}
}

/* USBL_COMMS_INIT_MODULE_H */
#endif
