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
#include <labust/sensors/image/ObjectDetector.hpp>

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
          void calculatePixMM() {
            pix_mm = (sonar_info.window_length - sonar_info.window_start) * 1000.0 / cartesian_img_size.height;
          }
          aris::SonarInfo sonar_info;
          cv::Size cartesian_img_size;
          bool polar;
          double pix_mm;
      };

      
      class Contours {
        public:
          
          class Contour {
            public:
              cv::vector<cv::Point> contour;
              double size;
              cv::Point center;
              bool operator < (const Contour other) const {
                return size < other.size;
              }    
              bool operator > (const Contour other) const {
                return size > other.size;
              }
          };
          
          Contours() {}
          Contours(cv::vector<cv::vector<cv::Point> > cntrs) {
            this->setContours(cntrs);
          }

          void setContours(cv::vector<cv::vector<cv::Point> > cntrs) {
            contours.resize(cntrs.size());
            for (int i=0; i<cntrs.size(); ++i) {
              contours[i].contour = cntrs[i];
              contours[i].size = cv::contourArea(contours[i].contour);
              cv::Moments mu = cv::moments(contours[i].contour, false);
              contours[i].center = cv::Point(mu.m10/mu.m00, mu.m01/mu.m00);
            }
          }

          void sort() {
            std::sort(contours.begin(), contours.end());
          }
          
          cv::vector<Contour> contours;
      };

      class SonarDetector: public ObjectDetector {
        public:
          SonarDetector() :
            blur_size(5),
            thresh_size(40),
            thresh_offset(-30),
            flood_fill_value(127) {}
         
          ~SonarDetector() {}
         
          virtual void detect(cv::Mat& image, cv::Point2f& center, double& area) {
            cvtColor(image, image, CV_BGR2GRAY);
            thresholdImage(image);
          }
        
        private:
          void thresholdImage(cv::Mat image) {
            // Flood fill outside of useful sonar image with neutral gray.
            cv::floodFill(image, cv::Point(1,1), flood_fill_value);
            cv::floodFill(image, cv::Point(image.cols-1, 1), flood_fill_value);
            cv::floodFill(image, cv::Point(1, image.rows-1), flood_fill_value);
            cv::floodFill(image, cv::Point(image.cols-1, image.rows-1), flood_fill_value);
          
            // Perform blurring to remove noise.
            cv::GaussianBlur(image, image, cv::Size(blur_size*2+1, blur_size*2+1), 0, 0, cv::BORDER_DEFAULT);
          
            // Threshold the image
            cv::adaptiveThreshold(image, image, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, thresh_size*2+3, thresh_offset);
          }

          void findInterestingRegions(cv::Mat image_thr) {
            // Find contours and sort them by size
            cv::vector<cv::vector<cv::Point> > cntrs;
            cv::vector<cv::Vec4i> hierarchy;
            cv::findContours(image_thr.clone(), cntrs, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE); 
            Contours contours(cntrs);
            contours.sort();

            // Cluster detected contours
            
          }

          cv::Scalar flood_fill_value;
          int blur_size, thresh_size, thresh_offset;

      };
    }
  }
}


/* SONARIMAGEUTILHPP_ */
#endif
