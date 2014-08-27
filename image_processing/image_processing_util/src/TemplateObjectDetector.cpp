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
#include <labust/sensors/image/TemplateObjectDetector.hpp>
#include <opencv2/opencv.hpp>


using namespace labust::sensors::image;

TemplateObjectDetector::TemplateObjectDetector() {
  WINDOW_ = "Template object detector";
}

TemplateObjectDetector::TemplateObjectDetector(const char *filename) {
  WINDOW_ = "Template object detector";
  templates_.push_back(cv::imread(filename, CV_LOAD_IMAGE_COLOR));
}

TemplateObjectDetector::TemplateObjectDetector(cv::Mat &img_template) {
  WINDOW_ = "Template object detector";
  templates_.push_back(img_template.clone());
}

void TemplateObjectDetector::addImageTemplate(const char *filename) {
  templates_.push_back(cv::imread(filename, CV_LOAD_IMAGE_COLOR));
}

void TemplateObjectDetector::addImageTemplate(cv::Mat &img_template) {
  templates_.push_back(img_template.clone());
}

void TemplateObjectDetector::setEnableVideoDisplay(bool enable_video_display) {
  enable_video_display_ = enable_video_display;
  this->createOpenCvWindow();
}

void TemplateObjectDetector::createOpenCvWindow() {
  cv::namedWindow(WINDOW_);
  cv::waitKey(1);
}

TemplateObjectDetector::~TemplateObjectDetector() {
  cv::destroyWindow(WINDOW_);
}

void TemplateObjectDetector::detect(cv::Mat &image, cv::Point2f &center, double &area) {
  double minVal, maxVal;
  double bestMaxVal = 0;
  cv::Point minLoc, bestMaxLoc, maxLoc;
  int bestMaxIndex;
  for (int i=0; i<templates_.size(); ++i) {
    cv::Mat result;
    cv::matchTemplate(image, templates_[i], result, CV_TM_CCOEFF_NORMED);
    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
    if (maxVal > bestMaxVal) {
      bestMaxVal = maxVal;
      bestMaxIndex = i;
      bestMaxLoc = maxLoc;
    }
  }
  center = bestMaxLoc + cv::Point(templates_[bestMaxIndex].rows/2, templates_[bestMaxIndex].cols/2);
  area = templates_[bestMaxIndex].rows * templates_[bestMaxIndex].cols;
}
