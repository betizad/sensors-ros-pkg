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
 *  Created: 24.05.2015.
 *********************************************************************/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <labust/sensors/image/ImageProcessingUtil.hpp>
#include <labust/sensors/image/ImageExtractor.hpp>

#include <fstream>

using namespace labust::sensors::image;


ImageExtractor::ImageExtractor() : 
    is_compressed(false),
    image_topic("/camera/image_raw"),
    filename_format("") {
  ros::NodeHandle ph("~"), nh("");
  ph.getParam("image_bag", image_bag);
  ph.getParam("image_topic", image_topic);
  ph.getParam("compressed", is_compressed);
  ph.getParam("filename_format", filename_format);
  this->onInit();
}

void ImageExtractor::onInit() {
  bag_reader.setBag(image_bag);
  bag_reader.addTopic(image_topic);
  bag_reader.open();
}

ImageExtractor::~ImageExtractor() {}

void ImageExtractor::start() {
  int frame = 0;
  char filename_buff[100];
  ROS_INFO("Starting image extraction.");
  while (!bag_reader.done()) {
    rosbag::MessageInstance m = bag_reader.nextMessageInstance();
    std::string filename;
    if (!filename_format.empty()) {
      sprintf(filename_buff, filename_format.c_str(), frame++);
      filename = filename_buff;
    }
    if (is_compressed) {
      std::ofstream wfile;
      sensor_msgs::CompressedImage::ConstPtr image_ptr = m.instantiate<sensor_msgs::CompressedImage>();
      if (filename.empty()) {
        std::stringstream ss;
        ss << m.getTime() << ".jpg";
        filename = ss.str();
      }
      wfile.open(filename.c_str(), std::ios::out | std::ios::binary);
      wfile.write((char*)&(image_ptr->data[0]), image_ptr->data.size() * sizeof(uint8_t));
      wfile.close();
    } else {
      sensor_msgs::Image::ConstPtr image_ptr = m.instantiate<sensor_msgs::Image>();
      cv_bridge::CvImagePtr cv_image = sensorImage2CvImage(image_ptr, "bgr8");
      if (filename.empty()) {
        std::stringstream ss;
        ss << m.getTime() << ".jpg";
        filename = ss.str();
      }
      cv::imwrite(filename.c_str(), cv_image->image.clone());
    }
  }
  ROS_INFO("Image extraction completed.");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_extractor");
  ImageExtractor image_extractor;
  image_extractor.start(); 

  return 0;
}
