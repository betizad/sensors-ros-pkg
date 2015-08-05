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
#ifndef VISUALORIENTATIONESTIMATOR_HPP_
#define VISUALORIENTATIONESTIMATOR_HPP_
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <labust/sensors/image/ObjectDetector.hpp>
#include <labust/sensors/image/ColorObjectDetector.hpp>
#include <labust/sensors/image/TemplateObjectDetector.hpp>

namespace labust {
  namespace sensors {
    namespace image {

      /**
       * Class for object orientation estimation based on familiar marker tracking.
       */
      template <class ObjDet>
      class VisualOrientationEstimator {

      public:
        VisualOrientationEstimator();

        VisualOrientationEstimator(ObjDet *obj_detector_1, ObjDet *obj_detector_2);

        void setObjectDetectors(ObjDet *obj_detector_1, ObjDet *obj_detector_2);

        void setReferences(const double ref_area_1, const double ref_area_2, 
            const double ref_pixel_size);
        void setReferences(const double ref_dist, const double ref_area_1, const double ref_area_2, 
            const double ref_pixel_size, const double ref_mutual_dist);

        void setMutalObjectDistance(const double ref_mutual_dist);

        ~VisualOrientationEstimator();

        void processFrame(cv::Mat frame);

      private:
        struct VisualMarker {
          cv::Point3f coordinate;
          cv::Point2f center;
          double depth, area;
          double ref_dist, ref_area, ref_pixel_size;
        };

        ObjDet* obj_detector_1, *obj_detector_2;
        VisualMarker vm_1, vm_2;
        double ref_mutual_distance;
      };
    }
  }
}

/* VISUALORIENTATIONESTIMATOR_HPP_ */
#endif
