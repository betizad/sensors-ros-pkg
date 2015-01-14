#ifndef IMUCOMPLEMENTARYQUATERNIONFILTER_HPP_
#define IMUCOMPLEMENTARYQUATERNIONFILTER_HPP_

#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>


namespace labust 
{
  namespace sensors 
  {
    /**
    * This class implements the complementary quaternion-based IMU filter 
    * based on paper by SOH Madgwick.
    * Paper: http://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf
    */
    class ImuComplementaryQuaternionFilter {

      public:
        /**
         * Main constructor.
         *  node_count - number of nodes (IMUs);
         *  dT - sampling period;
         *  beta - filter coefficient, consult paper for details;
         *  gyro_gain - scaling factor for converting gyro data to rad/s.
         */
        ImuComplementaryQuaternionFilter(const int node_count, const double dT, const double beta, const double gyro_zeta);
        
        /**
         * Generic destructor.
         */
        ~ImuComplementaryQuaternionFilter();
        
        void processFrame(const Eigen::MatrixXd data);
        std::vector<Eigen::Quaternion<double> > getQuaternionOrientation();

      private:
        void initialOrientation(const Eigen::MatrixXd data);
        const double dT, fs;
        const double beta, gyro_zeta;
        const int node_count;
        bool is_initialized;
        std::vector<Eigen::Quaternion<double> > q;
        double w_bx, w_by, w_bz;
    };
  }
}

/* IMUCOMPLEMENTARYQUATERNIONFILTER_HPP_ */
#endif
