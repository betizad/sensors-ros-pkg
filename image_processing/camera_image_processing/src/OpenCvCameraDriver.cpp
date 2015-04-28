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
 *  Created: 18.06.2014.
 *********************************************************************/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/image_encodings.h>

#include <labust/sensors/image/OpenCvCameraDriver.hpp>
#include <labust/sensors/image/ImageProcessingUtil.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <signal.h>

using namespace labust::sensors::image;

void sigint_handler(int sig) {
  // Sometimes does not get executed, glibc crashes instead.
  signal(SIGINT, SIG_DFL);
  ROS_ERROR("Segmentation fault, stopping camera.");
  ros::shutdown();
}

/**
 * ROS node for publishing videos from external cameras.
 */
OpenCvCameraDriver::OpenCvCameraDriver(ros::NodeHandle camera_nh, ros::NodeHandle ph) :
    ph_(ph),
    camera_nh_(camera_nh),
    it_(camera_nh_),
    image_pub_(it_.advertiseCamera("image_raw", 1)),
    camera_info_manager_(camera_nh_),
    camera_info_url_(""),
    is_video_(true),
    camera_address_("device"),
    device_id_(0) {
  ph.getParam("is_video", is_video_);
  ph.getParam("camera_address", camera_address_);
  ph.getParam("device_id", device_id_);
  ph.getParam("camera_info_url", camera_info_url_);
  if (camera_info_url_ != "") {
    camera_info_manager_.loadCameraInfo(camera_info_url_);
  }
}

OpenCvCameraDriver::~OpenCvCameraDriver() {
  if (is_video_) {
    video_capture_.release();
  }
}

void OpenCvCameraDriver::setup() {
  // If target is a valid video stream, open it now.
  if (is_video_) {
    if (camera_address_ == "device") {
      video_capture_.open(device_id_);
    } else {
      video_capture_.open(camera_address_);
    }
  }
}

void OpenCvCameraDriver::poll() {
  cv::Mat frame;
  // If target is a valid video stream, read one frame.
  // If target is not a valid video stream (e.g. a stream of .jpeg images), open image and read it.
  bool successful_read;
  if (is_video_) {
    successful_read = video_capture_.read(frame);
  } else {
    video_capture_.open(camera_address_);
    successful_read = video_capture_.read(frame);
  }
  if (!successful_read) {
    ROS_WARN("No frame");
  }
  // Convert cv::Mat image to sensor_msgs::Image and publish it.
  sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(camera_info_manager_.getCameraInfo()));
  image_pub_.publish(cvImage2SensorImage(frame, "bgr8"), ci);
  if (!is_video_) video_capture_.release();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "opencv_camera_driver_node");
  ros::NodeHandle camera_nh("camera"), ph("~");
  
  OpenCvCameraDriver ex_cam(camera_nh, ph);
  signal(SIGINT, &sigint_handler);
  ex_cam.setup();
  
  ros::Rate r(24);
  while (ros::ok()) {
    r.sleep();
    ex_cam.poll();
    ros::spinOnce();
  }

  return 0;
}
