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
#ifndef ARISSONAR_HPP_
#define ARISSONAR_HPP_
#include <ros/ros.h>
#include <aris/SonarInfo.h>
#include <aris/ARISConfig.h>
#include <opencv2/opencv.hpp>
#include <labust/sensors/image/SonarImageUtil.hpp>
#include <labust/sensors/image/ImageProcessingUtil.hpp>

#include <queue>


namespace labust {
  namespace sensors {
    namespace image {

      class ArisSonar {
        public:
          ArisSonar() {}
          ~ArisSonar() {}
         
          // Automatic range setting
          void setRangeOfInterest(double roi_start, double roi_end) {
            double new_window_start = sonar_info.window_start;
            double new_window_end = sonar_info.window_start + sonar_info.window_length;
            double frequency_high = sonar_info.frequency_hi;
            // Keep window_start between 50% and 10% of the roi_start
            if (sonar_info.window_start < 0.5 * roi_start ||
                sonar_info.window_start > 0.9 * roi_start) {
              new_window_start = 0.75 * roi_start;
            }
            if (sonar_info.window_start + sonar_info.window_length < 1.1 * roi_end ||
                sonar_info.window_start + sonar_info.window_length > 1.5 * roi_end) {
              new_window_end = 1.25 * roi_end;
            }
            
            if (frequency_high) {
              if (new_window_start < MIN_HIGH_RANGE) {
                new_window_start = MIN_HIGH_RANGE;
              }
              if (new_window_end > MAX_HIGH_RANGE) {
                frequency_high = false;
              }
            }
            if (!frequency_high) {
              if (new_window_start < MIN_LOW_RANGE) {
                new_window_start = MIN_LOW_RANGE;
              }
              if (new_window_end > MAX_LOW_RANGE) {
                new_window_end = MAX_LOW_RANGE;
                if (new_window_end < 0.75 * MAX_HIGH_RANGE) {
                  frequency_high = true;
                }
              }
            }

            setSonarRange(new_window_start, new_window_end);
            setSonarFrequencyHigh(frequency_high);
            uploadSonarConfig();
          }

          void setSonarRange(double start, double end) {
            sonar_cfg.request.window_start = start;
            sonar_cfg.request.window_length = end - start;
          }

          void setSonarFramePeriodSec(double frame_period_sec) {
            sonar_cfg.request.frame_period_sec = frame_period_sec;
          }

          void setSonarFrequencyHigh(bool high) {
            sonar_cfg.request.frequancy_hi = high;
          }

          void setSonarGain(double gain) {
            sonar_cfg.request.gain = gain;
          }

          void setSonarFocus(double focus) {
            sonar_cfg.request.focus = focus;
          }

          void setSonarSamplesPerBeam(int spb) {
            sonar_cfg.request.samples_per_beam = spb;
          }

          void uploadSonarConfig() {
            updateSonarConfigMessage();  
            if (pending_sonar_cfg.size() < 2) {
              pending_sonar_cfg.push(std::make_pair(sonar_cfg, -1));
            } else {
              pending_sonar_cfg.back().first = sonar_cfg;
            }
          } 

          void saveSonarInfo(aris::SonarInfo si) {
            processConfigQueue(si);
            sonar_info = si;
            sonar_cfg = getArisServiceMsg();
          }

          void saveSonarImage(const sensor_msgs::Image::ConstPtr &image) {
            sonar_cv_image = sensorImage2CvImage(image, sensor_msgs::image_encodings::BGR8);
          }

          aris::SonarInfo getSonarInfo() {
            return sonar_info;
          }

          cv_bridge::CvImagePtr getSonarImage() {
            return sonar_cv_image;
          }

          void saveCartesianImageSize(cv::Size sz) {
            cartesian_img_size = sz;
          }

          double getPixMM() {
            return pix_mm;
          }

          cv::Point2f pixelToCoordinate(cv::Point p) {
            return pix_mm * p;
          }
          
        private:
          aris::ARISConfig getArisServiceMsg() {
            aris::ARISConfig cfg;
            if (!pending_sonar_cfg.empty()) {
              cfg = pending_sonar_cfg.back().first;
            } else {
              cfg.request.frame_period_sec = 1.0 / sonar_info.frame_rate;
              cfg.request.gain = sonar_info.receiver_gain;
              cfg.request.frequancy_hi = sonar_info.frequency_hi;
              cfg.request.focus = sonar_info.focus;
              cfg.request.pulse_width = sonar_info.pulse_width;
              cfg.request.window_start = sonar_info.window_start * 1.0345;
              cfg.request.window_length = sonar_info.window_length * 1.045;
              cfg.request.samples_per_beam = sonar_info.samples_per_beam;
            }
            return cfg;
          }
          
          bool hasSonarInfoChanged(aris::SonarInfo new_sonar_info) {
            return !(sonar_info.window_start == new_sonar_info.window_start &&
                sonar_info.window_length == new_sonar_info.window_length &&
                sonar_info.frame_rate == new_sonar_info.frame_rate && 
                sonar_info.receiver_gain == new_sonar_info.receiver_gain &&
                sonar_info.focus == new_sonar_info.focus &&
                sonar_info.frequency_hi == new_sonar_info.frequency_hi &&
                sonar_info.pulse_width == new_sonar_info.pulse_width &&
                sonar_info.samples_per_beam == new_sonar_info.samples_per_beam && 
                sonar_info.frame_rate == new_sonar_info.frame_rate);
          }

          bool checkSonarConfigMessageIsValid() {
            return (sonar_cfg.request.frame_period_sec != 0 &&
                sonar_cfg.request.gain != 0 &&
                sonar_cfg.request.focus != 0 &&
                sonar_cfg.request.pulse_width != 0 &&
                sonar_cfg.request.window_start != 0 &&
                sonar_cfg.request.window_length != 0 &&
                sonar_cfg.request.samples_per_beam != 0);
          }

          void updateSonarConfigMessage() {
              if (sonar_cfg.request.gain == 0) sonar_cfg.request.gain = sonar_info.receiver_gain;
              if (sonar_cfg.request.focus == 0) sonar_cfg.request.focus = sonar_info.focus;
              if (sonar_cfg.request.pulse_width == 0) sonar_cfg.request.pulse_width = sonar_info.pulse_width;
              if (sonar_cfg.request.window_start == 0) sonar_cfg.request.window_start = sonar_info.window_start;
              if (sonar_cfg.request.window_length == 0) sonar_cfg.request.window_length = sonar_info.window_length;
              if (sonar_cfg.request.samples_per_beam == 0) sonar_cfg.request.samples_per_beam = sonar_info.samples_per_beam;
          }

          void processConfigQueue(aris::SonarInfo new_sonar_info) {
            if (pending_sonar_cfg.empty()) return;
            // If top of the queue is not processed, send it to sonar.
            if (pending_sonar_cfg.front().second < 0) {
              ros::NodeHandle nh;
              ros::ServiceClient client = nh.serviceClient<aris::ARISConfig>("/aris_configuration");
              updateSonarConfigMessage();
              if (!checkSonarConfigMessageIsValid()) {
                ROS_ERROR("Invalid sonar configuration data, possibly waiting for sonar info.");
                return;
              }
              if (client.call(pending_sonar_cfg.front().first)) {
                pending_sonar_cfg.front().second = 0;
                ROS_INFO("ARIS configuration sent successfully."); 
              } else {
                ROS_ERROR("Failed to set ARIS configuration parameters.");
                if (pending_sonar_cfg.front().second-- < -3) {
                  pending_sonar_cfg.pop();
                  ROS_ERROR("Maximum retry count reached, discarding configuration data.");
                };
              }
            // Else if sonar info has changed (meaning new parameters set) 
            // remove top from the queue as it has been processed and applied to the sonar.
            } else if (hasSonarInfoChanged(new_sonar_info)) {
              ROS_INFO("ARIS configuration applied.");
              pending_sonar_cfg.pop();
            } else if (pending_sonar_cfg.front().second > 10) {
              ROS_WARN("ARIS configuration maybe applied, but no change in parameters detected.");
              pending_sonar_cfg.pop();
            } else {
              pending_sonar_cfg.front().second++;
            }
          }

          void calculatePixMM() {
            pix_mm = (sonar_info.window_length - sonar_info.window_start) * 1000.0 / cartesian_img_size.height;
          }

          aris::SonarInfo sonar_info;
          aris::ARISConfig sonar_cfg;
          std::queue<std::pair<aris::ARISConfig, int> > pending_sonar_cfg;
          cv_bridge::CvImagePtr sonar_cv_image;
          cv::Size cartesian_img_size;
          bool polar;
          double pix_mm;
          static const double MAX_HIGH_RANGE = 7.0, MIN_HIGH_RANGE = 0.7;
          static const double MAX_LOW_RANGE = 20.0, MIN_LOW_RANGE = 0.7;
      };

    }
  }
}


/* ARISSONARHPP_ */
#endif
