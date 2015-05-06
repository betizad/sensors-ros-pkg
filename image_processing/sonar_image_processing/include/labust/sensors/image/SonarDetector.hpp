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
#ifndef SONARDETECTOR_HPP_
#define SONARDETECTOR_HPP_
#include <ros/ros.h>
#include <aris/SonarInfo.h>
#include <aris/ARISConfig.h>
#include <opencv2/opencv.hpp>

#include <labust/sensors/image/ObjectDetector.hpp>
#include <labust/sensors/image/ImageProcessingUtil.hpp>
#include <labust/sensors/image/SonarImageUtil.hpp>

#include <labust/navigation/KinematicModel.hpp>
#include <labust/navigation/KFCore.hpp>


namespace labust {
  namespace sensors {
    namespace image {

      typedef labust::navigation::KFCore<labust::navigation::KinematicModel> KFNav; 

      class SonarDetector: public ObjectDetector {
        public:
          SonarDetector() :
              blur_size(5),
              thresh_size(75),
              thresh_offset(25),
              flood_fill_value(127),
              is_initialized(false) {
            cv::namedWindow("thr");
            cv::createTrackbar("Blur size", "thr", &blur_size, 10);
            cv::createTrackbar("Thresh size", "thr", &thresh_size, 100);
            cv::createTrackbar("Thresh offset", "thr", &thresh_offset, 100);
          }
         
          ~SonarDetector() {}
         
          void setContourClusteringParams(double max_conn_dist, double min_cont_sz, double min_target_sz, double max_target_sz) {
            max_connected_distance = max_conn_dist;
            min_contour_size = min_cont_sz;
            min_target_size = min_target_sz;
            max_target_size = max_target_sz;
          }
          
          void setSonarInfo(aris::SonarInfo si) {
            if (si.window_start != sonar_info.window_start || si.window_length != sonar_info.window_length) {
              is_initialized = false;
            }
            if (si.frame_rate != sonar_info.frame_rate) {
              kf_estimator.setTs(1/si.frame_rate);
            }
            sonar_info = si;
          }

          virtual void detect(cv::Mat image, cv::Point2f& center, double& area) {
            cvtColor(image, image, CV_BGR2GRAY);
            if (!is_initialized) {
              recalculateBackgroundMask(image);
              roi = cv::Rect(pixToMM(cv::Point2f(0,0)), pixToMM(cv::Point2f(mask.cols, mask.rows)));
            }
            thresholdImage(image);
            
            cv::vector<cv::Rect> curr_rois = findInterestingRegions(image);
            updateRoi(curr_rois, center, area);
            
            labust::navigation::KFBase<labust::navigation::KinematicModel>::vectorref prediction = kf_estimator.getState(); 
            std::cout << "X: " << prediction[0] << std::endl;
            std::cout << "Y: " << prediction[1] << std::endl;
            std::cout << "VELOCITY: " << prediction[2] << std::endl;
            std::cout << "PSI: " << prediction[3] << std::endl;
            std::cout << "R: " << prediction[4] << std::endl;
            cv::Point2f center_estimate(prediction[0]*1000, prediction[1]*1000);
            cv::circle(image, MMToPix(center), 5, cv::Scalar(199), -1);
            cv::circle(image, MMToPix(center_estimate), 5, cv::Scalar(100), -1);
            cv::rectangle(image, rectMMToRectPix(roi), cv::Scalar(125), 3);
            cv::imshow("thr", image); cv::waitKey(1);
            double range, ber;
            range = sqrt(center.x*center.x + center.y*center.y);
            ber = 180/M_PI * std::atan2(center.x, center.y);
          }
        
        private:
          void recalculateBackgroundMask(cv::Mat image) {
            mask = cv::Mat::zeros(image.size()+cv::Size(2,2), CV_8UC1);
            cv::floodFill(image, mask, cv::Point(1,1), 255, 0, cv::Scalar(), cv::Scalar(),  4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);
            cv::floodFill(image, mask, cv::Point(image.cols-1, 1), 255, 0, cv::Scalar(), cv::Scalar(),  4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);
            cv::floodFill(image, mask, cv::Point(1, image.rows-1), 255, 0, cv::Scalar(), cv::Scalar(),  4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);
            cv::floodFill(image, mask, cv::Point(image.cols-1, image.rows-1), 255, 0, cv::Scalar(), cv::Scalar(),  4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);
            cv::Rect roi(cv::Point(2,2), image.size());
            mask = mask(roi);
            mask = cv::Scalar::all(255) - mask;
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(27,27), cv::Point(13,13));
            cv::erode(mask, mask, kernel);

            pix_mm = 1000.0 * sonar_info.window_length / image.rows; 
            is_initialized = true;
          }

          void thresholdImage(cv::Mat image) {
            // Flood fill outside of useful sonar image with neutral gray.
            cv::floodFill(image, cv::Point(1,1), flood_fill_value);
            cv::floodFill(image, cv::Point(image.cols-1, 1), flood_fill_value);
            cv::floodFill(image, cv::Point(1, image.rows-1), flood_fill_value);
            cv::floodFill(image, cv::Point(image.cols-1, image.rows-1), flood_fill_value);
            
            // Perform blurring to remove noise.
            cv::GaussianBlur(image, image, cv::Size(blur_size*2+1, blur_size*2+1), 0, 0, cv::BORDER_DEFAULT);
          
            // Threshold the image
            cv::adaptiveThreshold(image, image, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, thresh_size*2+3, -thresh_offset);
            image &= mask;
          }

          cv::vector<cv::Rect> findInterestingRegions(cv::Mat image_thr) {
            cv::vector<cv::Rect> roi_rects;
            // Find contours and sort them by size
            cv::vector<cv::vector<cv::Point> > cntrs;
            cv::vector<cv::Vec4i> hierarchy;
            cv::findContours(image_thr.clone(), cntrs, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE); 
            Contours contours(cntrs);
            contours.sort();

            // Cluster detected contours
            std::vector<std::vector<int> > clusters = contours.cluster(max_connected_distance/pix_mm, min_contour_size/(pix_mm*pix_mm));
            if (clusters.size() == 0) return roi_rects;
            int ind = 0;
            for (int i=0; i<clusters.size(); ++i) {
              int curr_cluster_size = 0;
              for (int j=0; j<clusters[i].size(); ++j) {
                curr_cluster_size += contours.contours[clusters[i][j]].size;
              }
              if (curr_cluster_size > min_target_size/(pix_mm*pix_mm) &&
                  curr_cluster_size < max_target_size/(pix_mm*pix_mm)) {
                clusters[ind++] = clusters[i];
              }
            }
            clusters.resize(ind);
          
            cv::Point2f center; 
            // Create ROI bounding rectangles for each of the candidate clusters
            for (int i=0; i<clusters.size(); ++i) {
              cv::Rect brect;
              double size = 0;
              for (int j=0; j<clusters[i].size(); ++j) {
                if (brect == cv::Rect()) {
                  brect = cv::boundingRect(contours.contours[clusters[i][j]].contour);
                } else {
                  brect |= cv::boundingRect(contours.contours[clusters[i][j]].contour);
                }
                size += contours.contours[clusters[i][j]].size;
              }
              if (mask.rows - 0.5*(brect.br().y + brect.tl().y) < 25) continue;
              if (brect.area() > max_target_size/(pix_mm*pix_mm)) continue;
              //cv::rectangle(image_thr, brect, cv::Scalar(170), 5);
              if (i==0) center = 0.5 * (brect.br() + brect.tl());
              roi_rects.push_back(cv::Rect(pixToMM(brect.tl()), pixToMM(brect.br())));
            }
            //cv::circle(image_thr, center, 5, cv::Scalar(199), -1);
            //cv::imshow("test", image_thr); cv::waitKey(1);
            return roi_rects;
          }

          void updateRoi(cv::vector<cv::Rect> curr_rois, cv::Point2f& center, double& area) {
            //
            kf_estimator.predict();
            labust::navigation::KFBase<labust::navigation::KinematicModel>::vectorref prediction = kf_estimator.getState(); 
            cv::Point2f prediction_center(prediction[0]*1000, prediction[1]*1000);
            cv::Point2f prediction_offset = 0.5*(roi.tl() + roi.br());
            prediction_offset = prediction_center - prediction_offset;
            roi.x += prediction_offset.x;
            roi.y += prediction_offset.y;

            cv::vector<cv::Rect> roi_candidates;
            for (int i=0; i<curr_rois.size(); ++i) {
              if (roi.contains(0.5 * (curr_rois[i].tl() + curr_rois[i].br()))) {
                roi_candidates.push_back(curr_rois[i]);     
              }
            }
            if (roi_candidates.size() > 0) {
              roi = roi_candidates[0];
              roi.x -= 0.25 * roi.width;
              roi.y -= 0.25 * roi.height;
              roi.width *= 1.5;
              roi.height *= 1.5;
              center = 0.5 * (roi_candidates[0].tl() + roi_candidates[0].br());
              area = roi_candidates[0].height * roi_candidates[0].width;
              labust::navigation::KFBase<labust::navigation::KinematicModel>::output_type correction(2);
              correction << center.x/1000, center.y/1000;
              kf_estimator.correct(correction); 
            } else {
              double m = std::max(roi.width, roi.height);
              roi.width = m;
              roi.height = m;
            }
            if (roi.width < 10000 || roi.height < 20000) {
              roi.x -= 0.05 * roi.width;
              roi.y -= 0.05 * roi.height;
              roi.width *= 1.1;
              roi.height *= 1.1;
            }
          }

          cv::Point2f pixToMM(const cv::Point2f p) {
            cv::Point2f temp(-p.x, p.y);
            cv::Point2f res = cv::Point2f(-mask.cols/2, mask.rows) - temp;
            return pix_mm * res + cv::Point2f(0, sonar_info.window_start*1000); 
          }

          cv::Point2f MMToPix(const cv::Point2f p) {
            return cv::Point2f(p.x/pix_mm + mask.cols/2, -(p.y-sonar_info.window_start*1000)/pix_mm + mask.rows);
          }

          cv::Rect rectPixToRectMM(const cv::Rect rect_pix) {
            return cv::Rect(pixToMM(rect_pix.tl()), pixToMM(rect_pix.br()));
          }

          cv::Rect rectMMToRectPix(const cv::Rect rect_mm) {
            return cv::Rect(MMToPix(rect_mm.tl()), MMToPix(rect_mm.br()));
          }

          aris::SonarInfo sonar_info;
          cv::Mat mask;
          cv::vector<cv::Rect> roi_rects;
          cv::Rect roi;
          KFNav kf_estimator;
          double pix_mm;
          cv::Scalar flood_fill_value;
          int blur_size, thresh_size, thresh_offset;
          double max_connected_distance, min_contour_size, min_target_size, max_target_size;
          bool is_initialized;
      };
    }
  }
}


/* SONARDETECTORHPP_ */
#endif
