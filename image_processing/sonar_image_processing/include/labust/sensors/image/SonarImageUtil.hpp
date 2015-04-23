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
#include <labust/sensors/image/ImageProcessingUtil.hpp>



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

          void saveSonarImage(const sensor_msgs::Image::ConstPtr &image) {
            sonar_cv_image = sensorImage2CvImage(image, sensor_msgs::image_encodings::BGR8);;
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
          void calculatePixMM() {
            pix_mm = (sonar_info.window_length - sonar_info.window_start) * 1000.0 / cartesian_img_size.height;
          }
          aris::SonarInfo sonar_info;
          cv_bridge::CvImagePtr sonar_cv_image;
          cv::Size cartesian_img_size;
          bool polar;
          double pix_mm;
      };

      class UnionFind {
        public:
          UnionFind(int N) {
            cnt = N;
            parent.resize(N);
            size.resize(N); 
            for (int i=0; i<N; ++i) {
              parent[i] = i;
              size[i] = 1;
            }
          }

          int count() {
            return cnt;
          }

          bool connected(int a, int b) {
            return find(a) == find(b);
          }

          int find(int a) {
            int root = a;
            while (root != parent[root]) root = parent[root];
            while (a != root) {
              int new_a = parent[a];
              parent[a] = root;
              a = new_a;
            }
            return root;
          }

          void unite(int a, int b) {
            int root_a = find(a);
            int root_b = find(b);
            if (root_a == root_b) return;
            if (size[root_a] < size[root_b] ) {
              parent[root_a] = root_b;
              size[root_b] += size[root_a];
            } else {
              parent[root_b] = root_a;
              size[root_a] += size[root_b];
            }
            cnt--;
          }

        private:
          std::vector<int> parent, size;
          int cnt;
      };
      
      class Contours {
        public:
          
          class Contour {
            public:
              cv::vector<cv::Point> contour;
              cv::Point2f center;
              double size;
              bool operator < (const Contour other) const {
                return size < other.size;
              }    
              bool operator > (const Contour other) const {
                return size > other.size;
              }
          };

          template <class First, class Second, class Third>
          class Triplet {
            public:
              Triplet(First a, Second b, Third c) :
                a(a), b(b), c(c) {}
              First a;
              Second b;
              Third c;
              bool operator < (const Triplet<First, Second, Third> other) {
                return a < other.a;
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
              cv::Rect brect = cv::boundingRect(contours[i].contour);
              contours[i].size = brect.height * brect.width;
              contours[i].center = 0.5 * (brect.tl() + brect.br());
            }
          }

          void sort() {
            std::sort(contours.begin(), contours.end(), std::greater<Contour>());
          }

          std::vector<std::vector<int> > cluster(double max_pix_dist, double min_contour_size) {
            std::vector<std::vector<int> > clusters;
            if (contours.size() ==0) return clusters; 
            int end = -1;
            while (contours[++end].size > min_contour_size);

            // Estimate mutual distances of contours
            std::vector<Triplet<double, int, int> > mutual_distances;
            for (int i=0; i<end; ++i) {
              for (int j=i+1; j<end; ++j) {
                double dist = sqrt( 
                    (contours[i].center.x - contours[j].center.x)*(contours[i].center.x - contours[j].center.x) +
                    (contours[i].center.y - contours[j].center.y)*(contours[i].center.y - contours[j].center.y) 
                  ) - (sqrt(contours[i].size/M_PI) + sqrt(contours[j].size/M_PI));
                if (dist < max_pix_dist) 
                  mutual_distances.push_back(Triplet<double, int, int>(dist, i, j));
              }
            }

            // Cluster the contours
            UnionFind uf(end);
            for (int i=0; i<mutual_distances.size(); ++i) {
              uf.unite(mutual_distances[i].b, mutual_distances[i].c);
            }
            int n_clusters = uf.count();
            std::vector<std::vector<int> > temp(end);
            clusters.resize(n_clusters);
            for (int i=0; i<end; ++i) {
              temp[uf.find(i)].push_back(i);
            }
            int curr_cluster = 0;
            for (int i=0; i<end; ++i) {
              if (temp[i].size() == 0) continue;
              clusters[curr_cluster++] = temp[i];
            }
            return clusters;
          }
          
          cv::vector<Contour> contours;
      };

      class SonarDetector: public ObjectDetector {
        public:
          SonarDetector() :
            blur_size(5),
            thresh_size(40),
            thresh_offset(-30),
            flood_fill_value(127),
            is_initialized(false) {}
         
          ~SonarDetector() {}
         
          void setContourClusteringParams(double max_conn_dist, double min_cont_sz, double min_roi_sz) {
            max_connected_distance = max_conn_dist;
            min_contour_size = min_cont_sz;
            min_roi_size = min_roi_sz;
          }
          
          void setSonarInfo(aris::SonarInfo si) {
            if (si.window_start != sonar_info.window_start || si.window_length != sonar_info.window_length) {
              is_initialized = false;
            }
            sonar_info = si;
          }

          virtual void detect(cv::Mat image, cv::Point2f& center, double& area) {
            cv::imshow("frame", image); cv::waitKey(1);
            cvtColor(image, image, CV_BGR2GRAY);
            if (!is_initialized) {
              recalculateBackgroundMask(image);
            }
            thresholdImage(image);
            findInterestingRegions(image);
          }
        
        private:
          void recalculateBackgroundMask(cv::Mat image) {
            mask = cv::Mat::zeros(image.size()+cv::Size(2,2), CV_8UC1);
            cv::floodFill(image, mask, cv::Point(1,1), flood_fill_value, 0, cv::Scalar(), cv::Scalar(),  4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);
            cv::floodFill(image, mask, cv::Point(image.cols-1, 1), flood_fill_value, 0, cv::Scalar(), cv::Scalar(),  4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);
            cv::floodFill(image, mask, cv::Point(1, image.rows-1), flood_fill_value, 0, cv::Scalar(), cv::Scalar(),  4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);
            cv::floodFill(image, mask, cv::Point(image.cols-1, image.rows-1), flood_fill_value, 0, cv::Scalar(), cv::Scalar(),  4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);
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
            cv::adaptiveThreshold(image, image, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, thresh_size*2+3, thresh_offset);
            image &= mask;
          }

          void findInterestingRegions(cv::Mat image_thr) {
            // Find contours and sort them by size
            cv::vector<cv::vector<cv::Point> > cntrs;
            cv::vector<cv::Vec4i> hierarchy;
            cv::findContours(image_thr.clone(), cntrs, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE); 
            Contours contours(cntrs);
            contours.sort();

            // Cluster detected contours
            std::vector<std::vector<int> > clusters = contours.cluster(max_connected_distance, min_contour_size);
            if (clusters.size() == 0) return;
            int ind = 0;
            for (int i=0; i<clusters.size(); ++i) {
              int curr_cluster_size = 0;
              for (int j=0; j<clusters[i].size(); ++j) {
                curr_cluster_size += contours.contours[clusters[i][j]].size;
              }
              if (curr_cluster_size > min_roi_size) {
                clusters[ind++] = clusters[i];
              }
            }
            clusters.resize(ind);
           
            // Create ROI bounding rectangles for each of the candidate clusters
            for (int i=0; i<clusters.size(); ++i) {
              cv::Rect brect;
              double size = 0;
              for (int j=0; j<clusters[i].size(); ++j) {
                std::cout << contours.contours[clusters[i].at(j)].contour.size() << std::endl;
                if (brect == cv::Rect()) {
                  brect = cv::boundingRect(contours.contours[clusters[i][j]].contour);
                } else {
                  brect |= cv::boundingRect(contours.contours[clusters[i][j]].contour);
                }
                size += contours.contours[clusters[i][j]].size;
              }
              cv::rectangle(image_thr, brect, cv::Scalar(170), 5);
              roi_rects.push_back(brect);
            }
            cv::imshow("test", image_thr); cv::waitKey(1);

          }

          aris::SonarInfo sonar_info;
          cv::Mat mask;
          cv::Scalar flood_fill_value;
          double pix_mm;
          cv::vector<cv::Rect> roi_rects;
          int blur_size, thresh_size, thresh_offset;
          double max_connected_distance, min_contour_size, min_roi_size;
          bool is_initialized;
      };
    }
  }
}


/* SONARIMAGEUTILHPP_ */
#endif
