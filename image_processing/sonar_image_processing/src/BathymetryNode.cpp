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
#include <std_msgs/Float64.h>
#include <sensor_msgs/image_encodings.h>

#include <labust/sensors/image/SonarImageUtil.hpp>
#include <labust/sensors/image/ArisSonar.hpp>
#include <labust/sensors/image/BathymetryNode.hpp>
#include <labust/sensors/image/ImageProcessingUtil.hpp>

#include <opencv2/opencv.hpp>


using namespace labust::sensors::image;


BathymetryNode::BathymetryNode() : 
    it(nh) {
  this->onInit();
};

BathymetryNode::~BathymetryNode() {};

void BathymetryNode::onInit() {
  ros::Rate rate(1);
  sonar_info_sub = nh.subscribe("/soundmetrics_aris3000/sonar_info", 1, &BathymetryNode::setSonarInfo, this);
  image_sub = it.subscribe("/soundmetrics_aris3000/polar", 1, &BathymetryNode::setSonarImage, this);
  sonar_bathymetry = nh.advertise<std_msgs::Float64MultiArray>("sonar_bathymetry", 1);
  sonar_altitude = nh.advertise<std_msgs::Float64>("sonar_altitude", 1);
}

void BathymetryNode::setSonarInfo(const aris::SonarInfo::ConstPtr &msg) {
  aris.saveSonarInfo(*msg);
  cv_bridge::CvImagePtr frame = aris.getSonarImage();
  if (frame == 0) return;
  if (frame->header.stamp == msg->header.stamp) {
    processFrame();
  }
}

void BathymetryNode::setSonarImage(const sensor_msgs::Image::ConstPtr &img) {
  aris.saveSonarImage(img);
  aris::SonarInfo si = aris.getSonarInfo();
  if (img->header.stamp == si.header.stamp) {
    processFrame();
  }
}

void BathymetryNode::processFrame() {
  cv_bridge::CvImagePtr cv_image_bgr = aris.getSonarImage();
  sonar_altitude_estimator.setSonarInfo(aris.getSonarInfo());
  std::vector<double> bath = sonar_altitude_estimator.process(cv_image_bgr->image.clone());
  double alt = *(std::min_element(bath.begin(), bath.end()));
  std_msgs::Float64MultiArrayPtr bathymetry(new std_msgs::Float64MultiArray());
  std_msgs::Float64Ptr altitude(new std_msgs::Float64);
  bathymetry->data.resize(bath.size());
  for (int i=0; i<bath.size(); ++i) {
    bathymetry->data[i] = bath[i];
  }
  altitude->data = alt;
  sonar_bathymetry.publish(bathymetry);
  sonar_altitude.publish(altitude);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "sonar_bathymetry_node"); 
  BathymetryNode node; 
  ros::spin();

  return 0;
}
