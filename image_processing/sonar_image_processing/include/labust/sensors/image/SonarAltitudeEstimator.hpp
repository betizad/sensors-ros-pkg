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
#ifndef SONARALTITUDEESTIMATOR_HPP_
#define SONARALTITUDEESTIMATOR_HPP_
#include <ros/ros.h>
#include <underwater_msgs/SonarInfo.h>
#include <underwater_msgs/ARISConfig.h>
#include <opencv2/opencv.hpp>
#include <labust/sensors/image/ImageProcessingUtil.hpp>
#include <labust/sensors/image/SonarImageUtil.hpp>


namespace labust {
  namespace sensors {
    namespace image {
      
      // Class for estimating altitude for eash of the sonar's beam
      // from ARIS sonar image.
      class SonarAltitudeEstimator {
        public:
          SonarAltitudeEstimator() :
              is_initialized(false) {}
         
          ~SonarAltitudeEstimator() {}
         
          // Stores the sonar info message.
          void setSonarInfo(underwater_msgs::SonarInfo si) {
            sonar_info = si;
          }

          // Main processing function.
          std::vector<double> process(cv::Mat image) {
            // Convert image to opencv grayscale.
            cvtColor(image, image, CV_BGR2GRAY);

            // Apply Gaussian blurring.
            cv::GaussianBlur(image, image, cv::Size(3*2+1, 3*2+1), 0, 0, cv::BORDER_DEFAULT);
            
            std::vector<double> res(image.cols);
            std::vector<int> val(image.cols);
            // For each of the beams (each beam is one column), find the max intensity pixel.
            for (int beam=0; beam<image.cols; ++beam) {
              uchar max = 0;
              double ind = 0;
              for (int sample=0; sample<image.rows; ++sample) {
                uchar curr = image.at<uchar>(sample, beam);
                if (curr > max) {max = curr; ind = sample;}
              }
	      
	            //if (max<100) ind = HUGE_VAL;
              // Calculate distance from pixel index.
              res[beam] = ind / image.rows * sonar_info.window_length + sonar_info.window_start;
              val[beam] = max;
            }
            // Adaptive bottom detection - experimental.
            // Samples must be above some minimum threshold or strong enough
            // compared to their neighbour (starting from the strongest one).
            std::vector<int>::iterator max_ind = std::max_element(val.begin(), val.end());
            int max_val = *max_ind;
            for (std::vector<int>::iterator ind = max_ind; ind != val.begin(); --ind) {
              if (!(*(ind-1) > 127 || static_cast<float>(*(ind-1)) / max_val > 0.75)) {
                res[ind-val.begin()-1] = HUGE_VAL;
              }
            }
            for (std::vector<int>::iterator ind = max_ind; ind != val.end(); ++ind) {
              if (!(*(ind+1) > 127 || static_cast<float>(*(ind+1)) / max_val > 0.9)) {
                res[ind-val.begin()+1] = HUGE_VAL;
              }
            }
            for (int i=0; i<image.cols; ++i) {
              cv::circle(image, cv::Point(i, (res[i] - sonar_info.window_start) / sonar_info.window_length * image.rows), 3, 255, -1);
            }
            //cv::imshow("img", image); cv::waitKey(100);
            return res;
          }

        private:
          underwater_msgs::SonarInfo sonar_info;
          double pix_mm;
          bool is_initialized;
      };
    }
  }
}


/* SONARALTITUDEESTIMATORHPP_ */
#endif
