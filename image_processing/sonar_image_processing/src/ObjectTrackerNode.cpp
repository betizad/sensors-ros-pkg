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
 *
 *  Author : Ivor Rendulic
 *  Created: 15.03.2015. 
 *********************************************************************/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/image_encodings.h>

#include <labust/sensors/image/SonarImageUtil.hpp>
#include <labust/sensors/image/SonarDetector.hpp>
#include <labust/sensors/image/ArisSonar.hpp>
#include <labust/sensors/image/ObjectTrackerNode.hpp>
#include <labust/sensors/image/ImageProcessingUtil.hpp>

#include <opencv2/opencv.hpp>

using namespace labust::sensors::image;

ObjectTrackerNode::ObjectTrackerNode() : 
    it(nh) {
  ros::NodeHandle ph("~");
  ph.getParam("target_size", target_size);
  this->onInit();
};

ObjectTrackerNode::~ObjectTrackerNode() {};

void ObjectTrackerNode::onInit() {
  ros::Rate rate(1);
  //usbl_fix_sub = nh.subscribe("/USBLFix", 1, &ObjectTrackerNode::adjustRangeFromUSBL, this);
  nav_filter_estimate_sub = nh.subscribe("/buddy/relative_position", 1, &ObjectTrackerNode::setNavFilterEstimate, this);
  position_sub = nh.subscribe("/buddy/position", 1, &ObjectTrackerNode::setHeading, this);
  sonar_info_sub = nh.subscribe("/soundmetrics_aris3000/sonar_info", 1, &ObjectTrackerNode::setSonarInfo, this);
  image_sub = it.subscribe("/soundmetrics_aris3000/cartesian", 1, &ObjectTrackerNode::setSonarImage, this);
  sonar_fix_pub = nh.advertise<underwater_msgs::SonarFix>("sonar_fix", 1);
  
  sonar_detector.setContourClusteringParams(
      100 /* max connected_distance in mm */, 
      100 /* min_contour_size in mm^2 */, 
      target_size /* target_size (area) in mm^2 */);
}

void ObjectTrackerNode::setSonarInfo(const aris::SonarInfo::ConstPtr &msg) {
  aris.saveSonarInfo(*msg);
  cv_bridge::CvImagePtr frame = aris.getSonarImage();
  if (frame == 0) return;
  if (frame->header.stamp == msg->header.stamp) {
    processFrame();
  }
}

void ObjectTrackerNode::setSonarImage(const sensor_msgs::Image::ConstPtr &img) {
  aris.saveSonarImage(img);
  aris::SonarInfo si = aris.getSonarInfo();
  if (img->header.stamp == si.header.stamp) {
    processFrame();
  }
}

void ObjectTrackerNode::setNavFilterEstimate(const navcon_msgs::RelativePosition& nav_filter_estimate) {
  sonar_detector.adjustROIFromFilterEstimate(nav_filter_estimate);
}

void ObjectTrackerNode::setHeading(const auv_msgs::NavSts& position_estimate) {
  sonar_detector.setHeading(position_estimate.orientation.yaw * 180 / M_PI);
}

void ObjectTrackerNode::adjustRangeFromUSBL(const underwater_msgs::USBLFix& usbl_fix) {
  aris.setRangeOfInterest(0.75 * usbl_fix.range, 1.5 * usbl_fix.range);
}

void ObjectTrackerNode::processFrame() {
  cv_bridge::CvImagePtr cv_image_bgr = aris.getSonarImage();
  cv::Point2f center;
  double area;
  aris.saveCartesianImageSize(cv_image_bgr->image.size());
  sonar_detector.setSonarInfo(aris.getSonarInfo());
  sonar_detector.detect(cv_image_bgr->image.clone(), center, area);
  double range, bearing;
  sonar_detector.getMeasuredRangeAndBearing(&range, &bearing);
  underwater_msgs::SonarFix sonar_fix;
  sonar_fix.range = range;
  sonar_fix.bearing = bearing;
  sonar_fix.header = aris.getSonarInfo().header;
  if (range > 0) {
    sonar_fix_pub.publish(sonar_fix);
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "sonar_object_tracker_node"); 
  ObjectTrackerNode node; 
  ros::spin();

  return 0;
}
