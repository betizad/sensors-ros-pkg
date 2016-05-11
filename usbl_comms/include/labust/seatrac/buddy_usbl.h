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
#ifndef USBL_COMMS_BUDDY_USBL_H
#define USBL_COMMS_BUDDY_USBL_H
#include <labust/seatrac/device_controller.h>
#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/pinger.h>
#include <labust/comms/caddy/caddy_messages.h>
#include <labust/seatrac/nav_module.h>
#include <labust/seatrac/buddy_payload.h>
#include <labust/seatrac/command_module.h>
#include <labust/seatrac/init_module.h>
#include <labust/seatrac/nav_handler.h>
#include <labust/comms/caddy/surface_handler.h>
#include <labust/comms/caddy/diver_handler.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <auv_msgs/NavSts.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>

#include <boost/thread.hpp>
#include <Eigen/Dense>

using labust::comms::caddy::SurfaceHandler;
using labust::comms::caddy::DiverHandler;

namespace labust
{
  namespace seatrac
  {
    /**
     * The class implements the status publisher and decoder.
     */
    class BuddyUSBL : virtual public DeviceController
    {
      typedef boost::function<void(const std::vector<uint8_t>& msg)> Functor;
      typedef std::map<int, Functor > HandlerMap;

      typedef message_filters::TimeSynchronizer<geometry_msgs::PointStamped, geometry_msgs::PointStamped> TimeSync;
      typedef boost::shared_ptr< TimeSync > TimeSyncPtr;
      typedef message_filters::Subscriber<geometry_msgs::PointStamped> FilterSub;
      typedef boost::shared_ptr< FilterSub > FilterSubPtr;

      enum {TIMEOUT=4, DIVER_ID=2, SURFACE_ID=1};
      enum {SURFACE=0, DIVER, AGENT_CNT};
    public:
      ///Main constructor
      BuddyUSBL();
      ///Default destructor
      ~BuddyUSBL();

      ///Listener configuration.
      bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph);

    private:
      ///The USBL pinger
      Pinger pinger;

      /// Handles reinitialization requests
      void onReinit(const geometry_msgs::PointStamped::ConstPtr& ned,
          const geometry_msgs::PointStamped::ConstPtr& llh);

      ///Handles incoming acoustic data.
      void onData(const labust::seatrac::DatReceive& data);
      //Handler the mode chaning topics.
      void onModeChange(const std_msgs::Bool::ConstPtr& msg);
      ///Handles the Buddy data
      void onSurfaceData(const std::vector<uint8_t>& data);
      ///Handles the Diver data
      void onDiverData(const std::vector<uint8_t>& data);
      /// The diver pinging method.
      void pingDiver(const labust::seatrac::DatSendCmd::Ptr& data);
      /// The surface pinging method.
      void pingSurface(const labust::seatrac::DatSendCmd::Ptr& data);
      /// Helper method to determine if Buddy position should be sent.
      bool sendPosition(int agent);
      /// Helper method to setup and start pinging
      void startPinging(bool flag = true);
      /// Helper method to reset the initialization.
      void resetInit();
      /// The main pinging function
      void run();

      /// Mode changing subscriber
      ros::Subscriber mode_sub;

      ///The next message mux.
      boost::mutex message_mux;
      ///The ping rate
      double ping_rate;
      ///Handlers for acoustic messages
      HandlerMap handlers;

      /// Initialization offset
      Eigen::Vector3d init_offset;
      /// The llh position if the init
      Eigen::Vector3d init_llh;

      /// The initialization module.
      InitModule init;
      /// Navigation module
      NavModule nav;
      /// Payload module
      BuddyPayload payload;
      /// The mission status module
      CommandModule status;

      /// The navigation data handler.
      SurfaceHandler surfacehandler;
      /// The navigation data handler.
      DiverHandler diverhandler;

      ///The worker thread
      boost::thread worker;
      /// The message assembly mutex
      boost::mutex message_assembly;
      ///The run flag for the worker thread.
      bool run_flag;
      ///The master pinger flag.
      bool is_master;

      /// The initialization flags
      bool inited[AGENT_CNT];
      /// The command confirmation flags
      int unconfirmed_cmd[AGENT_CNT];
      /// The timing flags
      ros::Time buddy_pos_update[AGENT_CNT];
    };
  }
}

/* USBL_COMMS_BUDDY_USBL_H */
#endif
