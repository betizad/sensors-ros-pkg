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
#ifndef BATHYMETRYNODE_HPP_
#define BATHYMETRYNODE_HPP_
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <labust/sensors/image/SonarImageUtil.hpp>
#include <labust/sensors/image/SonarAltitudeEstimator.hpp>

namespace labust {
  namespace sensors {
    namespace image {

      /**
       * ROS node for bathymetry calculation from sonar image.
       */
      class BathymetryNode {

      public:
        BathymetryNode();
        ~BathymetryNode();
      private:
        void onInit();
        void setSonarInfo(const aris::SonarInfo::ConstPtr &msg);
        void setSonarImage(const sensor_msgs::ImageConstPtr &img);
        void processFrame();
        void recalculateBearings(int nbeams);
        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        ros::Subscriber sonar_info_sub;
        ros::Publisher sonar_altitude_pub, sonar_bathymetry_pub;
        image_transport::Subscriber image_sub;
        ArisSonar aris;
        SonarAltitudeEstimator sonar_altitude_estimator;
        std::vector<double> bearing;
        double altitude_offset;
      };

    }
  }
}

/* BATHYMETRYNODE_HPP_ */
#endif
