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
#include <labust/sensors/image/ObjectTrackerNode.hpp>
#include <labust/sensors/image/ImageProcessingUtil.hpp>

#include <opencv2/opencv.hpp>


using namespace labust::sensors::image;

/**
 * Creates a ROS node for object tracking from sonar image.
 */
ObjectTrackerNode::ObjectTrackerNode() : 
    it(nh) {
  this->onInit();
};

ObjectTrackerNode::~ObjectTrackerNode() {};

void ObjectTrackerNode::onInit() {
  ros::Rate rate(1);
  sonar_info_sub = nh.subscribe("/soundmetrics_aris3000/sonar_info", 1, &ObjectTrackerNode::setSonarInfo, this);
  image_sub = it.subscribe("/soundmetrics_aris3000/cartesian", 1, &ObjectTrackerNode::setSonarImage, this);
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

void ObjectTrackerNode::processFrame() {
  cv_bridge::CvImagePtr cv_image_bgr = aris.getSonarImage();
  cv::Point2f center;
  double area;
  aris.saveCartesianImageSize(cv_image_bgr->image.size());
  sonar_detector.setContourClusteringParams(50,10,200);
  sonar_detector.setSonarInfo(aris.getSonarInfo());
  sonar_detector.detect(cv_image_bgr->image.clone(), center, area);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "diver_tracker_node"); 
  ObjectTrackerNode node; 
  ros::spin();

  return 0;
}
