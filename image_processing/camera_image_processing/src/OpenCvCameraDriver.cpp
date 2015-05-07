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

size_t write_data(void* ptr, size_t size, size_t nmemb, sensor_msgs::CompressedImage::_data_type* stream) {
  uint8_t* pt = reinterpret_cast<uint8_t*>(ptr);
  stream->insert(stream->end(),pt, pt + (size*nmemb));
  return size*nmemb;
}


/**
 * ROS node for publishing videos from external cameras.
 */
OpenCvCameraDriver::OpenCvCameraDriver(ros::NodeHandle camera_nh, ros::NodeHandle ph) :
    ph_(ph),
    camera_nh_(camera_nh),
    it_(camera_nh_),
    camera_info_manager_(camera_nh_),
    camera_info_url_(""),
    is_video_(true),
    compressed_only_(false),
    curl(0),
    camera_address_("device"),
    device_id_(0) {
  ph.getParam("is_video", is_video_);
  ph.getParam("compressed_only", compressed_only_);
  ph.getParam("camera_address", camera_address_);
  ph.getParam("device_id", device_id_);
  ph.getParam("camera_info_url", camera_info_url_);
  
  if (!compressed_only_) {
    image_pub_ = it_.advertiseCamera("image_raw", 1);
  } else {
    compressed_image_pub_ = camera_nh_.advertise<sensor_msgs::CompressedImage>("/camera/image_raw/compressed", 1);
    compressed_camera_info_pub_ = camera_nh_.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1);
  };
      
  if (camera_info_url_ != "") {
    camera_info_manager_.loadCameraInfo(camera_info_url_);
  }
  
  curl = curl_easy_init();
  if (curl == 0) {
    ROS_ERROR("Failed to init Curl.");
  }
  curl_easy_setopt(curl, CURLOPT_URL, camera_address_.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
}

OpenCvCameraDriver::~OpenCvCameraDriver() {
  if (is_video_) {
    video_capture_.release();
  }
  if (curl) {
    curl_easy_cleanup(curl);
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
  
  if (!compressed_only_) {
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
    sensor_msgs::ImagePtr sensor_image = cvImage2SensorImage(frame, "bgr8");
    image_pub_.publish(sensor_image, ci);
    if (!is_video_) video_capture_.release();
  }
  if (compressed_only_ && !is_video_) {
    sensor_msgs::CompressedImagePtr compressed_image(new sensor_msgs::CompressedImage());
    compressed_image->format = "bgr8";
    compressed_image->format += "; jpeg compressed";
    
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &compressed_image->data);
    CURLcode res = curl_easy_perform(curl);

    compressed_image_pub_.publish(compressed_image);
    compressed_camera_info_pub_.publish(camera_info_manager_.getCameraInfo());
  }
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
