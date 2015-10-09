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

      const int DEFAULT_BINARIZATION_THRESHOLD_SIZE = 75;
      const int DEFAULT_BINARIZATION_THRESHOLD_OFFSET = 25;
      const int DEFAULT_BLUR_SIZE = 5;

      const double DEG_TO_RAD = M_PI/180;
      const double RAD_TO_DEG = 180/M_PI;

      typedef labust::navigation::KFCore<labust::navigation::KinematicModel> KFNav; 

      class SonarDetector: public ObjectDetector {
        public:
          SonarDetector() :
              blur_size(DEFAULT_BLUR_SIZE),
              thresh_size(DEFAULT_BINARIZATION_THRESHOLD_SIZE),
              thresh_offset(DEFAULT_BINARIZATION_THRESHOLD_OFFSET),
              is_kf_initialized(false),
              is_detector_initialized(false),
              frames_without_measurement(0) {
            //cv::namedWindow("Tracking");
            //cv::createTrackbar("Blur size", "Tracking", &blur_size, 10);
            //cv::createTrackbar("Thresh size", "Tracking", &thresh_size, 100);
            //cv::createTrackbar("Thresh offset", "Tracking", &thresh_offset, 100);
           
            KFNav::vector q(5);
            q << std::pow(0.5,2), //x
                 std::pow(0.5,2), //y
                 std::pow(0.25,2), //u
                 std::pow(0.1,2), //psi
                 std::pow(0.1,2); //r 
            KFNav::matrix W = KFNav::matrix::Identity(5,5);
            KFNav::matrix Q = q.asDiagonal();
            kf_estimator.setStateParameters(W, Q);
          }
         
          ~SonarDetector() {}
         
          /**
           * Set target and contour clustering parameters:
           *  max_conn_dist - all countours closer than this will be clustered together
           *  min_cont_sz - ignore contours smaller than this size
           *  target_sz - expected size of the target; favor clusters closer to this size
           */
          void setContourClusteringParams(double max_conn_dist, double min_cont_sz, double target_sz) {
            max_connected_distance = max_conn_dist;
            min_contour_size = min_cont_sz;
            target_size = target_sz;
          }
         
          /**
           * Save sonar info. Recalculate parameters if range or frame rate changes.
           */ 
          void setSonarInfo(aris::SonarInfo si) {
            // Re-initialize the detector if the range changes.
            if (si.window_start != sonar_info.window_start || si.window_length != sonar_info.window_length) {
              is_detector_initialized = false;
            }

            // Re-intialize the KF estimator if the frame rate changes.
            if (si.frame_rate != sonar_info.frame_rate) {
              kf_estimator.setTs(1/si.frame_rate);
              is_kf_initialized = false;
            }

            // Heading compensation from sonar's compass.
            if (!is_detector_initialized) {
              delta_heading = 0;
            } else {
              delta_heading = si.compass_heading - sonar_info.compass_heading;
            }
            sonar_info = si;
          }

          /**
           * Perform object detection.
           */
          virtual void detect(cv::Mat image, cv::Point2f& center, double& area) {
            cv::Mat img_temp;
            image.copyTo(img_temp);
            cvtColor(image, image, CV_BGR2GRAY);
            if (!is_detector_initialized) {
              recalculateBackgroundMask(image);
              roi = cv::Rect(pixToMM(cv::Point2f(0,0)), pixToMM(cv::Point2f(mask.cols, mask.rows)));
            }
            thresholdImage(image);
             
            cv::vector<cv::Rect> curr_rois = findInterestingRegions(image);
            updateRoi(curr_rois, center, area);
            
            KFNav::vectorref prediction = kf_estimator.getState(); 
            cv::Point2f center_estimate(prediction[0]*1000, prediction[1]*1000);
            //cv::circle(image, MMToPix(center), 5, cv::Scalar(199), -1);
            //cv::circle(image, MMToPix(center_estimate), 5, cv::Scalar(100), -1);
            //cv::rectangle(image, rectMMToRectPix(roi), cv::Scalar(125), 3);
            //cv::rectangle(img_temp, rectMMToRectPix(roi), cv::Scalar(0,0,255), 3);
            //cv::imshow("Tracking", image); cv::waitKey(10);
            //cv::imshow("Sonar tracking", img_temp); cv::waitKey(10);
            estimated_range = sqrt(center_estimate.x*center_estimate.x + center_estimate.y*center_estimate.y) / 1e3;
            estimated_bearing = std::atan2(center_estimate.x, center_estimate.y);
            measurement_range = sqrt(center.x*center.x + center.y*center.y) / 1e3;
            measurement_bearing = std::atan2(center.x, center.y);
          }

          void getMeasuredRangeAndBearing(double* range, double* bearing) {
            *range = measurement_range;
            *bearing = measurement_bearing;
          }

          void getEstimatedRangeAndBearing(double* range, double* bearing) {
            *range = estimated_range;
            *bearing = estimated_bearing;
          }
        
        private:
          /**
           * Calculate background mask to remove artifacts on visible area limits.
           */
          void recalculateBackgroundMask(cv::Mat image) {
            mask = cv::Mat::zeros(image.size()+cv::Size(2,2), CV_8UC1);
            cv::floodFill(image, mask, cv::Point(1,1), 255, 0, 
                          cv::Scalar(), cv::Scalar(),  4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);
            cv::floodFill(image, mask, cv::Point(image.cols-1, 1), 255, 0, 
                          cv::Scalar(), cv::Scalar(),  4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);
            cv::floodFill(image, mask, cv::Point(1, image.rows-1), 255, 0, 
                          cv::Scalar(), cv::Scalar(),  4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);
            cv::floodFill(image, mask, cv::Point(image.cols-1, image.rows-1), 255, 0, 
                          cv::Scalar(), cv::Scalar(),  4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);
            cv::Rect roi(cv::Point(2,2), image.size());
            mask = mask(roi);
            mask = cv::Scalar::all(255) - mask;
            cv::Mat kernel = cv::getStructuringElement(
                cv::MORPH_RECT, cv::Size(27,27), cv::Point(13,13));
            cv::erode(mask, mask, kernel);

            pix_mm = 1000.0 * sonar_info.window_length / image.rows; 
            is_detector_initialized = true;
          }

          /**
           * Perform adaptive thresholding.
           */
          void thresholdImage(cv::Mat image) {
            // Flood fill outside of useful sonar image with neutral gray
            // to improve thresholding results.
            cv::floodFill(image, cv::Point(1,1), 127);
            cv::floodFill(image, cv::Point(image.cols-1, 1), 127);
            cv::floodFill(image, cv::Point(1, image.rows-1), 127);
            cv::floodFill(image, cv::Point(image.cols-1, image.rows-1), 127);
            
            // Perform blurring to remove noise.
            cv::GaussianBlur(image, image, cv::Size(blur_size*2+1, blur_size*2+1), 
                             0, 0, cv::BORDER_DEFAULT);
          
            // Threshold the image
            cv::adaptiveThreshold(image, image, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 
                                  thresh_size*2+3, -thresh_offset);
            image &= mask;
          }

          /**
           * Detect candidates for object.
           */
          cv::vector<cv::Rect> findInterestingRegions(cv::Mat image_thr) {
            cv::vector<cv::Rect> roi_rects;
            // Find contours and sort them by size
            cv::vector<cv::vector<cv::Point> > cntrs;
            cv::vector<cv::Vec4i> hierarchy;
            cv::findContours(image_thr.clone(), cntrs, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE); 
            ContourProcessor contour_processor(cntrs);
            contour_processor.sort();

            // Cluster detected contours
            std::vector<std::vector<int> > clusters = contour_processor.cluster(
                max_connected_distance/pix_mm, min_contour_size/(pix_mm*pix_mm));
            if (clusters.size() == 0) return roi_rects;
            int ind = 0;
            for (int i=0; i<clusters.size(); ++i) {
              int curr_cluster_size = 0;
              for (int j=0; j<clusters[i].size(); ++j) {
                curr_cluster_size += contour_processor.contours[clusters[i][j]].size;
              }
              if (curr_cluster_size * pix_mm * pix_mm < 0.1 * target_size || 
                  curr_cluster_size * pix_mm * pix_mm > 2 * target_size) continue;
              clusters[ind++] = clusters[i];
            }
            clusters.resize(ind);
          
            cv::Point2f center; 
            // Create ROI bounding rectangles for each of the candidate clusters
            for (int i=0; i<clusters.size(); ++i) {
              cv::Rect brect;
              double size = 0;
              for (int j=0; j<clusters[i].size(); ++j) {
                if (brect == cv::Rect()) {
                  brect = cv::boundingRect(contour_processor.contours[clusters[i][j]].contour);
                } else {
                  brect |= cv::boundingRect(contour_processor.contours[clusters[i][j]].contour);
                }
                size += contour_processor.contours[clusters[i][j]].size;
              }
              if (mask.rows - 0.5*(brect.br().y + brect.tl().y) < 25) continue;
              if (size < 0.1 * target_size/1e6) continue;
              if (i==0) center = 0.5 * (brect.br() + brect.tl());
              roi_rects.push_back(cv::Rect(pixToMM(brect.tl()), pixToMM(brect.br())));
            }
            return roi_rects;
          }

          /**
           * Find the best candidate corresponding to the model prediction and update model.
           */
          void updateRoi(cv::vector<cv::Rect> curr_rois, cv::Point2f& center, double& area) {
            // Move the estimator state according to heading change
            // Ignore heading changes less than 1 degree.
            if (std::abs(delta_heading) > 1) {
              KFNav::vector state = kf_estimator.getState();
              KFNav::matrix rotation_matrix(2,2);
              rotation_matrix(0,0) = std::cos(-M_PI/180 * delta_heading); 
              rotation_matrix(0,1) = -std::sin(-M_PI/180 * delta_heading); 
              rotation_matrix(1,0) = std::sin(-M_PI/180 * delta_heading); 
              rotation_matrix(1,1) = std::cos(-M_PI/180 * delta_heading); 
              KFNav::vector position(2);
              position << state[0], state[1];
              position = rotation_matrix * position;
              state[0] = position[0];
              state[1] = position[1];
              kf_estimator.setState(state);
            }
            
            // Move and scale the ROI based on estimator prediction
            kf_estimator.predict();
            KFNav::vectorref prediction = kf_estimator.getState(); 
            KFNav::matrixref state_covariance = kf_estimator.getStateCovariance();
            if (prediction[1] != 0) {
              cv::Point2f prediction_center(prediction[0]*1000.0, prediction[1]*1000.0);
              cv::Size roi_size(sqrt(target_size) * 3 * pow(1.1, frames_without_measurement),
                                sqrt(target_size) * 3 * pow(1.1, frames_without_measurement));
              roi = cv::Rect(prediction_center, roi_size);
              // ROI area dependent on estimator covariance. This didn't work that well.
              /*roi = cv::Rect(prediction_center, 
                             cv::Size(sqrt(target_size)+10*1000.0*sqrt(state_covariance(0,0)), 
                                      sqrt(target_size)+10*1000.0*sqrt(state_covariance(1,1))));*/
              roi = moveRect(roi, roi.tl());
            }
            
            // Move roi to sonar visible area only
            cv::Rect visible_area = rectPixToRectMM(cv::Rect(cv::Point2f(0,0), 
                                                    cv::Point2f(mask.cols, mask.rows)));
            roi &= visible_area;
            if (!visible_area.contains(rectCenter(roi))) {
              roi = visible_area;
            }
            //
            cv::vector<cv::Rect> roi_candidates;
            for (int i=0; i<curr_rois.size(); ++i) {
              if (roi.contains(rectCenter(curr_rois[i]))) {
                roi_candidates.push_back(curr_rois[i]);     
              }
            }
            if (roi_candidates.size() == 0) {
              if (frames_without_measurement < 50) frames_without_measurement++;
            } else {
              frames_without_measurement -= 2;
              if (frames_without_measurement < 0) frames_without_measurement = 0;
            }
            if (roi_candidates.size() > 0) {
              double best_value = HUGE_VAL;
              double best_roi_variance;
              int best_roi_ind;
              for (int i=0; i<roi_candidates.size(); ++i) {
                area = roi_candidates[i].area();
                double curr_roi_dist = distanceBetweenPoints(rectCenter(roi_candidates[i]), rectCenter(roi));
                double curr_roi_stdev = 1 - exp(-(0.001*(area-target_size) * 0.001*(area-target_size) / (2 << 20)));
                if (curr_roi_dist * curr_roi_stdev < best_value) {
                  best_value = curr_roi_dist * sqrt(abs(target_size - roi_candidates[i].area()));
                  best_roi_ind = i;
                  best_roi_variance = curr_roi_stdev * curr_roi_stdev;
                }
              }
              center = rectCenter(roi_candidates[best_roi_ind]);
              area = roi_candidates[best_roi_ind].area();
              KFNav::output_type correction(2);
              
              // Set measurement variance; empirical.
              kf_estimator.setMeasurementParameters(KFNav::matrix::Identity(2,2), 
                  (10*best_roi_variance + 0.1) * KFNav::matrix::Identity(2,2));
              correction << center.x/1000.0, center.y/1000.0;
              kf_estimator.correct(correction); 
              
              // If this is the first measurement, set state to it.
              if (!is_kf_initialized) {
                KFNav::vector state(5);
                state << center.x/1000.0, center.y/1000.0, 0.0, 0.0, 0.0;
                kf_estimator.setState(state);
                is_kf_initialized = true;
              }
            }
          }

 
          cv::Point2f pixToMM(const cv::Point2f p) {
            cv::Point2f temp(-p.x, p.y);
            cv::Point2f res = cv::Point2f(-mask.cols/2, mask.rows) - temp;
            return pix_mm * res + cv::Point2f(0, sonar_info.window_start*1000); 
          }

          cv::Point2f MMToPix(const cv::Point2f p) {
            return cv::Point2f(p.x/pix_mm + mask.cols/2, 
                               -(p.y-sonar_info.window_start*1000)/pix_mm + mask.rows);
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
          double delta_heading;
          double pix_mm;
          double measurement_range, measurement_bearing;
          double estimated_range, estimated_bearing;
          int blur_size, thresh_size, thresh_offset;
          int frames_without_measurement;
          double max_connected_distance, min_contour_size, target_size;
          bool is_detector_initialized, is_kf_initialized;
      };
    }
  }
}


/* SONARDETECTORHPP_ */
#endif
