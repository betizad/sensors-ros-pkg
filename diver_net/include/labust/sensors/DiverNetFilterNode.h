#ifndef DIVERNETFILTERNODE_H_
#define DIVERNETFILTERNODE_H_
#include <ros/ros.h>
#include <labust/sensors/ImuFilter.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
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
        void setCalibrationRequest(const std_msgs::Bool::ConstPtr& calibrate);
        /**
         * Initializes gyro mean calculation. Next N frames (default N=150)
         * the gyro data will be summmed and the gyro_bias matrix will be 
         * populated with mean values of the gyro. Callback from
         * calculate_gyro_mean topic.
         */
        void calculateGyroMean(const std_msgs::Bool::ConstPtr& gyro_mean);
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
        
        ros::NodeHandle ph_, nh_;
        // ROS subscribers for data, calibration request and gyro mean calculation request.
        ros::Subscriber raw_data_, calibrate_sub_, gyro_mean_sub_;
        // ROS publishers for raw and filtered rpy angles.
        ros::Publisher raw_angles_pub_, filtered_angles_pub_;
        // ROS publishers for raw and filtered quaternions.
        ros::Publisher filtered_quaternion_pub_;
        // Arrays holding raw and filtered rpy angles.
        std_msgs::Float64MultiArrayPtr rpy_raw_, rpy_filtered_, quaternion_filtered_;
        // Vector with axes permutation of the DiverNet.
        std::vector<Eigen::Matrix3d> axes_permutation_;
        // Holds mean gyro values.
        Eigen::MatrixXd gyro_bias_;
        // Holds magnetomter calibration values.
        Eigen::MatrixXd magnetometer_ellipsoid_center_;
        Eigen::MatrixXd magnetometer_ellipsoid_scale_;
        bool should_calibrate_magnetometer_;
        std::vector<ImuFilter> filters_;
        // Number of DiverNet nodes.
        const int node_count_;
        // Number of frames for gyro mean calculation.
        const int gyro_mean_num_frames_;
        // Number of frames until gyro mean calculation is done.
        int gyro_mean_calculation_frames_left_;
        // Stores request for pose calibration.
        bool should_calibrate_pose_;
    };
  }
}

/* DIVERNETFILTERNODE_H_ */
#endif
