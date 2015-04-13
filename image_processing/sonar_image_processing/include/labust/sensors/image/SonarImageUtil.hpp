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
#ifndef SONARIMAGEUTIL_HPP_
#define SONARIMAGEUTIL_HPP_
#include <ros/ros.h>
#include <aris/SonarInfo.h>
#include <aris/ARISConfig.h>
#include <opencv2/opencv.hpp>

namespace labust {
  namespace sensors {
    namespace image {

      class ArisSonar {
        public:
          ArisSonar() {}
          ~ArisSonar() {}
         
          void setSonarRange(double start, double end) {
            aris::ARISConfig cfg = getArisServiceMsg();
            cfg.request.window_start = start;
            cfg.request.window_length = end - start;
            setSonarParameters(cfg);              
          }

          void setSonarFrequency(bool high) {
            aris::ARISConfig cfg = getArisServiceMsg();
            cfg.request.frequancy_hi = high;
            setSonarParameters(cfg);              
          }

          void setSonarGain(double gain) {
            aris::ARISConfig cfg = getArisServiceMsg();
            cfg.request.gain = gain;
            setSonarParameters(cfg);              
          }

          void setSonarSamplesPerBeam(int spb) {
            aris::ARISConfig cfg = getArisServiceMsg();
            cfg.request.samples_per_beam = spb;
            setSonarParameters(cfg);              
          }

          aris::ARISConfig getArisServiceMsg() {
            aris::ARISConfig cfg;
            cfg.request.frame_period_sec = sonar_info.sample_period;
            cfg.request.gain = sonar_info.receiver_gain;
            cfg.request.frequancy_hi = sonar_info.frequency_hi;
            cfg.request.focus = sonar_info.focus;
            cfg.request.pulse_width = sonar_info.pulse_width;
            cfg.request.window_start = sonar_info.window_start;
            cfg.request.window_length = sonar_info.window_length;
            cfg.request.samples_per_beam = sonar_info.samples_per_beam;
            return cfg;
          }

          void setSonarParameters(aris::ARISConfig cfg) {
            ros::NodeHandle nh;
            ros::ServiceClient client = nh.serviceClient<aris::ARISConfig>("/aris_configuration");
            if (client.call(cfg)) {
              ROS_INFO("ARIS configuration changed successfully."); 
            } else {
              ROS_ERROR("Failed to set ARIS configuration parameters.");
            }
          } 

          void saveSonarInfo(aris::SonarInfo si) {
            sonar_info = si;
          }

          double getPixMM() {
            return pix_mm;
          }

          cv::Point pixelToCoordinate(cv::Point p) {

          }
          
        private:
          /*void recalculateParams() {
            calculatePixMM();
          }

          void calculatePixMM() {
            pix_mm = (params.window_length - params.window_start) * 1000.0 / params.iysize;
          }*/
          aris::SonarInfo sonar_info;
          bool polar;
          double pix_mm;
      };
     


      typedef cv::vector<std::pair<double, cv::vector<cv::Point> > > contourtype;

      bool compareContoursAndArea(const std::pair<double, cv::vector<cv::Point> > a, const std::pair<double, cv::vector<cv::Point> > b) {
          return (a.first < b.first);
      }

      cv::vector<cv::vector<cv::Point> > findAndSortContours(cv::Mat image, bool sort=true, int mode=CV_RETR_EXTERNAL, int method=CV_CHAIN_APPROX_NONE) {
        cv::vector<cv::vector<cv::Point> > contours;
        cv::vector<cv::Vec4i> hierarchy;
        cv::findContours(image, contours, hierarchy, mode, method, cv::Point(0,0));
        if (sort) {
          contourtype contoursAndArea(contours.size());
          for (int i=0; i<contours.size(); ++i) {
            contoursAndArea[i].second = contours[i];
            contoursAndArea[i].first = cv::contourArea(contours[i]);
          }
          std::sort(contoursAndArea.begin(), contoursAndArea.end(), compareContoursAndArea);
          for (int i=0; i<contours.size(); ++i) {
            contours[i] = contoursAndArea[i].second;
          }
        }
        return contours;
      }

      cv::vector<cv::vector<cv::Point> > findContours(cv::Mat image, int mode=CV_RETR_EXTERNAL, int method=CV_CHAIN_APPROX_NONE) {
        return findAndSortContours(image, false, mode, method);
      }


    }
  }
}


/* SONARIMAGEUTILHPP_ */
#endif
