#ifndef DIVERNETFILTERNODE_HPP_
#define DIVERNETFILTERNODE_HPP_
#include <ros/ros.h>
#include <labust/sensors/ImuComplementaryQuaternionFilter.hpp>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>

namespace labust 
{
  namespace sensors 
  {
    /**
    * This class implements the filtering node for DiverNet.
    * Both raw and filtered orientation is calculated and published.
    * Madgwick's complementary quaternion-based filter is used.
    */
    class DiverNetFilterNode {

      public:
        /**
         * Main constructor
         */
        DiverNetFilterNode();
        /**
         * Generic destructor.
         */
        ~DiverNetFilterNode();

      private:
        void onInit();
        void processData(const std_msgs::Int16MultiArrayPtr &raw_data);
        void calculateRawAngles(const Eigen::MatrixXd raw);
        ros::Subscriber raw_data;
        ros::Publisher raw_angles_publisher, filtered_angles_publisher;
        std_msgs::Float64MultiArrayPtr rpy_raw, rpy_filtered;
        ImuComplementaryQuaternionFilter *filter;
        const double dT, fs;
        int node_count, data_per_node;
    };
  }
}

/* DIVERNETFILTERNODE_HPP_ */
#endif
