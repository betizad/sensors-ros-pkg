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
#include <aris/SonarInfo.h>
#include <aris/ARISConfig.h>
#include <opencv2/opencv.hpp>
#include <labust/sensors/image/ImageProcessingUtil.hpp>
#include <labust/sensors/image/SonarImageUtil.hpp>


namespace labust {
  namespace sensors {
    namespace image {

      class SonarAltitudeEstimator {
        public:
          SonarAltitudeEstimator() :
              is_initialized(false) {}
         
          ~SonarAltitudeEstimator() {}
         
          void setSonarInfo(aris::SonarInfo si) {
            sonar_info = si;
          }

          std::vector<double> process(cv::Mat image) {
            cvtColor(image, image, CV_BGR2GRAY);
            cv::GaussianBlur(image, image, cv::Size(10*2+1, 10*2+1), 0, 0, cv::BORDER_DEFAULT);
            std::vector<double> res(image.cols);
            for (int beam=0; beam<image.cols; ++beam) {
              uchar max = 0;
              int ind = 0;
              for (int sample=0; sample<image.rows; ++sample) {
                uchar curr = image.at<uchar>(sample, beam);
                if (curr > max) {max = curr; ind = sample;}
              }
              res[beam] = ind * sonar_info.window_length / image.rows;
            }
            return res;
          }

        private:
          aris::SonarInfo sonar_info;
          double pix_mm;
          bool is_initialized;
      };
    }
  }
}


/* SONARALTITUDEESTIMATORHPP_ */
#endif
