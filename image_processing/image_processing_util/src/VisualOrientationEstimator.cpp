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
 *  Created: 27.01.2015.
 *********************************************************************/
#include <labust/sensors/image/VisualOrientationEstimator.hpp>


using namespace labust::sensors::image;

template <class ObjDet>
VisualOrientationEstimator<ObjDet>::VisualOrientationEstimator() : 
    obj_detector_1(0),
    obj_detector_2(0) {}

template <class ObjDet>
VisualOrientationEstimator<ObjDet>::VisualOrientationEstimator(ObjDet *obj_detector_1, ObjDet *obj_detector_2) :
    obj_detector_1(obj_detector_1),
    obj_detector_2(obj_detector_2) {}

template <class ObjDet>
void VisualOrientationEstimator<ObjDet>::setObjectDetectors(ObjDet *obj_detector_1, ObjDet *obj_detector_2) {
  this->obj_detector_1 = obj_detector_1;
  this->obj_detector_2 = obj_detector_2;
}

template <class ObjDet>
void VisualOrientationEstimator<ObjDet>::setReferences(const double ref_area_1, const double ref_area_2,
    const double ref_pixel_size) {
  this->setReferences(1.0, ref_area_1, ref_area_2, ref_pixel_size, 0.0);
}

template <class ObjDet>
void VisualOrientationEstimator<ObjDet>::setReferences(const double ref_dist, 
    const double ref_area_1, const double ref_area_2, const double ref_pixel_size, 
    const double ref_mutual_dist) {
  vm_1.ref_dist = ref_dist;
  vm_2.ref_dist = ref_dist;
  vm_1.ref_area = ref_area_1;
  vm_2.ref_area = ref_area_2;
  vm_1.ref_pixel_size = ref_pixel_size;
  vm_2.ref_pixel_size = ref_pixel_size;
  ref_mutual_distance = ref_mutual_dist;
}

template <class ObjDet>
VisualOrientationEstimator<ObjDet>::~VisualOrientationEstimator() {}

template <class ObjDet>
void VisualOrientationEstimator<ObjDet>::processFrame(cv::Mat frame) {
  obj_detector_1->detect(frame, vm_1.center, vm_1.area);
  obj_detector_2->detect(frame, vm_2.center, vm_2.area);
  vm_1.center -= cv::Point2f(frame.cols/2, frame.rows/2);
  vm_2.center -= cv::Point2f(frame.cols/2, frame.rows/2);

  // Estimate depth of each VisualMarker
  vm_1.depth = vm_1.ref_dist * sqrt(vm_1.ref_area / vm_1.area);
  vm_2.depth = vm_2.ref_dist * sqrt(vm_2.ref_area / vm_2.area);

  std::cout << std::endl << "CENTER 1 : " << vm_1.center << "  CENTER 2 : " << vm_2.center << std::endl;
  std::cout << std::endl << "DEPTH 1 : " << vm_1.depth << "  DEPTH 2 : " << vm_2.depth << std::endl << std::endl;

  // Relative coordinates are obtained either by knowing constant mutual distance,
  // or by esimating them from pixel sizes and positions in image
  if (ref_mutual_distance > 0) {
    vm_1.coordinate = cv::Point3f(0,0,vm_1.depth);
    // Reference fixed mutual distance allows us to calculate circle radius on which the (x,y) coordinate lies
    double radius = sqrt(ref_mutual_distance*ref_mutual_distance - (vm_2.depth - vm_1.depth)*(vm_2.depth - vm_1.depth));
    cv::Point2f vm_2_temp_center = vm_2.center - vm_1.center;
    double scale_factor = sqrt((vm_2_temp_center.x * vm_2_temp_center.x + vm_2_temp_center.y * vm_2_temp_center.y) / (radius*radius));
    vm_2.coordinate = cv::Point3f(vm_2_temp_center.x / scale_factor, vm_2_temp_center.y / scale_factor, vm_2.depth);
    std::cout << "RADIUS : " << radius  << std::endl;
    std::cout << "COORDINATES : " << vm_1.coordinate << " " << vm_2.coordinate << std::endl;
  } else {
  
  }

  // Calculate yaw and pitch angles; roll cannot be calculated based on two markers
  // TODO: add option of third marker?
  double yaw = std::atan2(vm_2.coordinate.z-vm_1.coordinate.z, vm_2.coordinate.x - vm_1.coordinate.x);
  double zz = vm_2.coordinate.z - vm_1.coordinate.z;
  double xx = vm_2.coordinate.x - vm_1.coordinate.x;
  double pitch = std::atan2(vm_2.coordinate.y-vm_1.coordinate.y, sqrt(zz*zz + xx*xx));
  std::cout << "ORIENTATION : PITCH : " << pitch << " YAW  : " << yaw << std::endl << std::endl;
}

template class VisualOrientationEstimator<ColorObjectDetector>;
template class VisualOrientationEstimator<TemplateObjectDetector>;
