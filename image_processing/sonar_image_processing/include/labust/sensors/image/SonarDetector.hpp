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
#include <underwater_msgs/SonarInfo.h>
#include <underwater_msgs/ARISConfig.h>
#include <opencv2/opencv.hpp>

#include <navcon_msgs/RelativePosition.h>

#include <labust/sensors/image/ObjectDetector.hpp>
#include <labust/sensors/image/ImageProcessingUtil.hpp>
#include <labust/sensors/image/SonarImageUtil.hpp>


namespace labust {
  namespace sensors {
    namespace image {

      const int DEFAULT_BINARIZATION_THRESHOLD_SIZE = 75;
      const int DEFAULT_BINARIZATION_THRESHOLD_OFFSET = 25;
      const int DEFAULT_BLUR_SIZE = 5;

      const double DEG_TO_RAD = M_PI/180;
      const double RAD_TO_DEG = 180/M_PI;

      class SonarDetector: public ObjectDetector {
        public:
          SonarDetector() :
              blur_size_(DEFAULT_BLUR_SIZE),
              threshold_size_(DEFAULT_BINARIZATION_THRESHOLD_SIZE),
              threshold_offset_(DEFAULT_BINARIZATION_THRESHOLD_OFFSET),
              frames_without_measurement(0),
              is_detector_initialized(false),
              enable_visualization_(false),
              reject_multiple_targets_(false),
              range_only_distance_(false),
              target_distance_threshold_(1000),
              roi_size_gain(3.0) {
            ros::NodeHandle ph("~");
            ph.param("roi_size_gain", roi_size_gain, roi_size_gain);
          }
         
          ~SonarDetector() {}
        
          void setEnableVisualization(bool enable_visualization) {
            enable_visualization_ = enable_visualization;
          }

          void startDebugWindow() {
            cv::namedWindow("Tracking");
            cv::createTrackbar("Blur size", "Tracking", &blur_size_, 10);
            cv::createTrackbar("Thresh size", "Tracking", &threshold_size_, 100);
            cv::createTrackbar("Thresh offset", "Tracking", &threshold_offset_, 100);
            cv::createTrackbar("Target size", "Tracking", &target_size, 2000000); 
          }
 
          /**
           * Set target and contour clustering parameters:
           *  max_conn_dist - all countours closer than this will be clustered together
           *  min_cont_sz - ignore contours smaller than this size
           */
          void setContourClusteringParams(int max_conn_dist, int min_cont_sz) {
            max_connected_distance = max_conn_dist;
            min_contour_size = min_cont_sz;
          }

          void setBinarizationParams(int blur_size, int threshold_size, int threshold_offset) {
            blur_size_ = blur_size;
            threshold_size_ = threshold_size;
            threshold_offset_ = threshold_offset;
          }

          /**
           * Sets approximate target area size (in mm^2).
           *  target_sz - expected size of the target; favor clusters closer to this size
           */
          void setTargetSize(int target_sz) {
            target_size = target_sz;
          }

          void setRejectMultipleTargets(bool reject_multiple_targets) {
            reject_multiple_targets_ = reject_multiple_targets;
          }

          void setRangeOnlyDistance(bool range_only_distance) {
            range_only_distance_ = range_only_distance;
          }

          void setTargetDistanceThreshold(int target_distance_threshold) {
            target_distance_threshold_ = target_distance_threshold;
          }
         
          /**
           * Save sonar info. Recalculate parameters if range or frame rate changes.
           */ 
          void setSonarInfo(underwater_msgs::SonarInfo si) {
            // Re-initialize the detector if the range changes.
            if (si.window_start != sonar_info.window_start || si.window_length != sonar_info.window_length) {
              is_detector_initialized = false;
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
            if (enable_visualization_) {
              visualize(image);
            }
            cvtColor(image, image, CV_BGR2GRAY);
            if (!is_detector_initialized) {
              recalculateBackgroundMask(image);
              // This line initializes the ROI to the entire sonar FOV.
              // TODO: agree if this is the desired behaviour or we shouldn't have ROI if not 
              // explicitly set.
              roi = cv::Rect(pixToMM(cv::Point2f(0,0)), pixToMM(cv::Point2f(mask.cols, mask.rows)));
            }
            thresholdImage(image);
             
            cv::vector<cv::Rect> curr_rois = findInterestingRegions(image);
            center = processCandidates(curr_rois);          
            measurement_range = sqrt(center.x*center.x + center.y*center.y) / 1e3;
            measurement_bearing = std::atan2(center.x, center.y);
            
            cv::circle(image, MMToPix(center), 5, cv::Scalar(199), -1);
            cv::rectangle(image, rectMMToRectPix(roi), cv::Scalar(125), 3);
            cv::rectangle(img_temp, rectMMToRectPix(roi), cv::Scalar(0,0,255), 3);
            if (enable_visualization_) {
              cv::imshow("Tracking", image); cv::waitKey(10);
              cv::imshow("Sonar tracking", img_temp); cv::waitKey(10);
            }
          }

          void getMeasuredRangeAndBearing(double* range, double* bearing) {
            *range = measurement_range;
            *bearing = measurement_bearing;
          }

          void getEstimatedRangeAndBearing(double* range, double* bearing) {
            *range = estimated_range;
            *bearing = estimated_bearing;
          }

          void setHeading(const double new_heading) {
            heading = new_heading;
          }

          void adjustROIFromFilterEstimate(const navcon_msgs::RelativePosition& filter_estimate) {
            // x-y inverted compared to filter; TODO: SonarDetector works in NED x-y system
            cv::Point roi_center(filter_estimate.y * 1000, filter_estimate.x * 1000);
            cv::Size roi_size(sqrt(target_size) + roi_size_gain*1000 * sqrt(filter_estimate.y_variance),
                              sqrt(target_size) + roi_size_gain*1000 * sqrt(filter_estimate.x_variance));
            //cv::Size roi_test_size(2000, 2000);
            roi = cv::Rect(roi_center, roi_size);
            roi = moveRect(roi, roi.tl());
            std::cerr << roi << std::endl;
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
            cv::GaussianBlur(image, image, cv::Size(blur_size_*2+1, blur_size_*2+1), 
                             0, 0, cv::BORDER_DEFAULT);
          
            // Threshold the image
            cv::adaptiveThreshold(image, image, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 
                                  threshold_size_*2+3, -threshold_offset_);
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

          cv::Point2f processCandidates(cv::vector<cv::Rect>& target_candidates) {
            int j=0;
            for (int i=0; i<target_candidates.size(); ++i) {
              if (roi.contains(rectCenter(target_candidates[i]))) {
                if (i!=j) {
                 target_candidates[j] = target_candidates[i];
                }
                j++;
              }
            }
            target_candidates.resize(j);
            if (target_candidates.size() == 0) {
              if (frames_without_measurement < 50) frames_without_measurement++;
            } else {
              frames_without_measurement -= 2;
              if (frames_without_measurement < 0) frames_without_measurement = 0;
            }
            if (target_candidates.size() == 0 || 
                (reject_multiple_targets_ && target_candidates.size() != 1)) return cv::Point2f(0,0);
            double best_value = HUGE_VAL;
            int best_roi_ind;
            double curr_roi_dist;
            for (int i=0; i<target_candidates.size(); ++i) {
              if (range_only_distance_) {
                curr_roi_dist = abs(distanceBetweenPoints(rectCenter(roi), cv::Point(0,0)) -
                                    distanceBetweenPoints(rectCenter(target_candidates[i]), cv::Point(0,0)));
              } else {
                curr_roi_dist = distanceBetweenPoints(rectCenter(target_candidates[i]), rectCenter(roi));
              }
              if (curr_roi_dist < best_value) {
                best_value = curr_roi_dist;
                best_roi_ind = i;
              }
            }
            if (best_value < target_distance_threshold_) {
              return rectCenter(target_candidates[best_roi_ind]);
            } else {
              return cv::Point(0,0);
            }
          }

          void visualize(const cv::Mat& frame) {
            if (!is_detector_initialized) return;
            int num_pix = (sonar_info.window_start + sonar_info.window_length) * 1000 / pix_mm * 2;
            cv::Mat image(num_pix, num_pix, CV_8UC3);
            image = cv::Scalar(0,0,0);
            cv::Rect area_in_image(cv::Point2f(num_pix/2 - frame.cols/2, 0),
                                   cv::Size(frame.cols, frame.rows));
            cv::Mat image_area = image(area_in_image);
            frame.copyTo(image_area);
            cv::rectangle(image, rectMMToRectPix(roi)+cv::Point2i(num_pix/2 - frame.cols/2), CV_RGB(255,0,0), 3);
            cv::resize(image, image, cv::Size(768, 768));
            
            /*(cv::Mat buddy = cv::imread("/home/ivor/buddy.png");
            cv::resize(buddy, buddy, cv::Size(64,92));
            area_in_image = cv::Rect(cv::Point(image.cols/2 - buddy.cols/2, image.rows/2 - buddy.rows/2),
                                     buddy.size());
            image_area = image(area_in_image);
            buddy.copyTo(image_area);*/

            cv::Mat rot_mat = cv::getRotationMatrix2D(cv::Point(image.cols/2, image.rows/2), -heading, 1.0);
            cv::warpAffine(image, image, rot_mat, image.size());

            cv::imshow("Sonar", image); cv::waitKey(100); 
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

          underwater_msgs::SonarInfo sonar_info;
          cv::Mat mask;
          cv::vector<cv::Rect> roi_rects;
          cv::Rect roi;
          double pix_mm;
          double measurement_range, measurement_bearing;
          double estimated_range, estimated_bearing;
          double heading, delta_heading;
          double roi_size_gain;
          int blur_size_, threshold_size_, threshold_offset_;
          int frames_without_measurement;
          int max_connected_distance, min_contour_size;
          int target_size; 
          int target_distance_threshold_;
          bool is_detector_initialized;
          bool enable_visualization_;
          bool reject_multiple_targets_;
          bool range_only_distance_;
      };
    }
  }
}


/* SONARDETECTORHPP_ */
#endif
