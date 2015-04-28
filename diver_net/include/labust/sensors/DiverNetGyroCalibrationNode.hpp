#ifndef DIVERNETGYROCALIBRATIONNODE_HPP_
#define DIVERNETGYROCALIBRATIONNODE_HPP_
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>

namespace labust 
{
  namespace sensors 
  {
    /**
    */
    class DiverNetGyroCalibrationNode {

      public:
        /**
         * Main constructor
         */
        DiverNetGyroCalibrationNode();
        /**
         * Generic destructor.
         */
        ~DiverNetGyroCalibrationNode();

      private:
        void processData(const std_msgs::Int16MultiArrayPtr &raw_data);
        ros::Subscriber raw_data;
        Eigen::MatrixXd gyro_bias;
        std::string calibration_file_;
        const int node_count, elem_count, max_frames;
        int n;
    };
  }
}

/* DIVERNETGYROCALIBRATIONNODE_HPP_ */
#endif
