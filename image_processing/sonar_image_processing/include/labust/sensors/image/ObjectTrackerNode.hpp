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
#ifndef OBJECTTRACKERNODE_HPP_
#define OBJECTTRACKERNODE_HPP_
#include <ros/ros.h>
#include <labust/sensors/image/SonarImageUtil.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <auv_msgs/NavSts.h>
#include <navcon_msgs/RelativePosition.h>
#include <sensor_msgs/Image.h>
#include <underwater_msgs/SonarFix.h>
#include <underwater_msgs/USBLFix.h>


namespace labust {
  namespace sensors {
    namespace image {

      /**
       * ROS node for object tracking from sonar image.
       */
      class ObjectTrackerNode {

      public:
        ObjectTrackerNode();
        ~ObjectTrackerNode();
      private:
        void onInit();
        void adjustRangeFromUSBL(const underwater_msgs::USBLFix& usbl_fix);
        void setNavFilterEstimate(const navcon_msgs::RelativePosition& nav_filter_estimate);
        void setHeading(const auv_msgs::NavSts& position_estimate);
        void setSonarInfo(const underwater_msgs::SonarInfo::ConstPtr &msg);
        void setSonarImage(const sensor_msgs::ImageConstPtr &img);
        void processFrame(const std::string& frame_id);
        ros::NodeHandle nh;
        ros::Subscriber sonar_info_sub, usbl_fix_sub, nav_filter_estimate_sub, position_sub;
        ros::Publisher sonar_fix_pub;
        image_transport::ImageTransport it;
        image_transport::Subscriber image_sub;
        ArisSonar aris;
        SonarDetector sonar_detector;
      };

    }
  }
}

/* OBJECTTRACKERNODE_HPP_ */
#endif
