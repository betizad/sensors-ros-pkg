#ifndef IMUCOMPLEMENTARYQUATERNIONFILTER_HPP_
#define IMUCOMPLEMENTARYQUATERNIONFILTER_HPP_

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>

namespace labust 
{
  namespace sensors 
  {
    /**
    * This class implements the complementary quaternion-based IMU filter designed by SOH Madgwick.
    */
    class ImuComplementaryQuaternionFilter {

      public:
        /**
         * Main constructor
         */
        ImuComplementaryQuaternionFilter(const int num_nodes, const double dT, const double beta, const double gyro_gain);
        /**
         * Generic destructor.
         */
        ~ImuComplementaryQuaternionFilter();
        
        void processFrame(const Eigen::MatrixXd data);
        std::vector<std::vector<double> > getQuaternionOrientation();
        void printQuaternionOrientation();

      private:
        void initialOrientation(const Eigen::MatrixXd data);
        const double dT, fs;
        const double beta, gyro_gain;
        const int num_nodes;
        bool is_initialized;
        std::vector<std::vector<double> > q;
    };
  }
}

/* IMUCOMPLEMENTARYQUATERNIONFILTER_HPP_ */
#endif
