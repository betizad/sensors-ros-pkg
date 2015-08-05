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
#include <labust/sensors/image/ImageProcessingUtil.hpp>


namespace labust {
  namespace sensors {
    namespace image {

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
            if (contours.size() == 0) return clusters; 
            int end = 0;
            while (end < contours.size() && contours[end].size > min_contour_size) end++;

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

    }
  }
}


/* SONARIMAGEUTILHPP_ */
#endif
