#ifndef DIVERNETFILTERNODE_H_
#define DIVERNETFILTERNODE_H_
#include <ros/ros.h>
#include <labust/sensors/ImuFilter.h>
#include <labust/sensors/MagnetometerCalibration.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <boost/circular_buffer.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

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
        enum CalibrationStatus {
          None, Off, Pending, On
        };
        /**
         * Called from constructor. Sets variables, subscribers and publishers,
         * initial axes permutation.
         */
        void onInit();
        /**
         * Loads the axes permutation for the model from a text file.
         */
        void loadModelAxesPermutation();
        /**
         * Loads previously calculated gyro bias from a file.
         */
        void loadGyroBiasFromFile();
        /**
         * Loads magnetometer calibration data.
         */
        void loadMagnetometerCalibration();
        /**
         * Processes data packet from DiverNet.
         */
        void processData(const std_msgs::Int16MultiArrayPtr &raw_data);
        /**
         * Sets request for pose calibration. Callback from calibrate topic.
         */
        void setPoseCalibrationRequest(const std_msgs::Int16::ConstPtr& pose_cal);
        /**
         * Initializes gyro mean calculation. Next N frames (default N=150)
         * the gyro data will be summmed and the gyro_bias matrix will be 
         * populated with mean values of the gyro. Callback from
         * calculate_gyro_mean topic.
         */
        void calculateGyroMean(const std_msgs::Int16::ConstPtr& gyro_mean);
        void setMagnetometerCalibrationRequest(const std_msgs::Int16::ConstPtr& mag_cal);
        /**
         * Calculates angles just from the accelerometer data. This is the 
         * initial step for the filtering process.
         */
        void calculateRawAngles(const Eigen::MatrixXd raw);
        /**
         * Calculates sensor permutation and adjusts axes_permutation. Called
         * when calibration request is received.
         */
        void calibratePose(const std::vector<Eigen::Quaternion<double> >& q);
        void calculateMotionRate();
        void calculateMagnetometerCalibration();

        ros::NodeHandle ph_, nh_;
        // ROS subscribers for data, calibration request and gyro mean calculation request.
        ros::Subscriber raw_data_, pose_cal_sub_, gyro_mean_sub_, mag_cal_sub_;
        // ROS publishers for raw and filtered rpy angles.
        ros::Publisher raw_angles_pub_, filtered_angles_pub_;
        // ROS publishers for raw and filtered quaternions.
        ros::Publisher filtered_quaternion_pub_;
        // Arrays holding raw and filtered rpy angles.
        std_msgs::Float64MultiArrayPtr rpy_raw_, rpy_filtered_, quaternion_filtered_;
        // Vector with axes permutation of the DiverNet.
        std::vector<Eigen::Matrix3d> axes_permutation_, pose_cal_permutation_;
        // Holds mean gyro values.
        Eigen::MatrixXd gyro_bias_;
        std::vector<ImuFilter> filters_;
        // Number of DiverNet nodes.
        const int node_count_;
        // Number of frames for gyro mean calculation.
        const int gyro_mean_num_frames_;
        // Number of frames until gyro mean calculation is done.
        int gyro_mean_calculation_frames_left_;
        // Stores request for pose calibration.
        bool should_calibrate_pose_;
        
        // Motion rate
        ros::Publisher motion_rate_pub_;
        boost::circular_buffer<double> data_buffer_[20][9], 
          hp_data_buffer_[20][9], 
          hp_magnitude_buffer_[20][3], 
          hp_lp_magnitude_buffer_[20][3];
        std::vector<int> motion_rate_nodes_;

        // Magnetometer calibration
        std::vector<Eigen::MatrixXd> magnetometer_buffer_;
        std::vector<MagnetometerCalibrationData> magnetometer_calibration_data_;
        int magnetometer_buffer_points_;
        bool enable_mag_cal_;
        CalibrationStatus mag_cal_status_, pose_cal_status_, gyro_mean_status_;
    };
  }
}

/* DIVERNETFILTERNODE_H_ */
#endif
