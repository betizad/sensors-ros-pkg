#include <labust/sensors/DiverNetFilterNode.hpp>
#include <labust/sensors/ImuComplementaryQuaternionFilter.hpp>
#include <labust/tools/conversions.hpp>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <fstream>

using namespace labust::sensors;

DiverNetFilterNode::DiverNetFilterNode():
    node_count(20),
    gyro_bias(20,3),
    should_calibrate(false),
    gyro_mean_calculation_frames_left(0),
    gyro_mean_num_frames(150),
    rpy_raw(new std_msgs::Float64MultiArray()),
    rpy_filtered(new std_msgs::Float64MultiArray()), 
    quaternion_filtered(new std_msgs::Float64MultiArray()),
    filter(new ImuComplementaryQuaternionFilter(20, /* num nodes */
                                                1.0/10, /*dT*/ 
                                                0.05, 0.1)) {
  this->onInit();
}

void DiverNetFilterNode::onInit() {
  ros::NodeHandle nh, ph("~");
  ros::Rate rate(1);

  calibrate_sub = nh.subscribe<std_msgs::Bool>(
      "calibrate", 1, &DiverNetFilterNode::setCalibrationRequest, this);
  gyro_mean_sub = nh.subscribe<std_msgs::Bool>(
      "calculate_gyro_mean", 1, &DiverNetFilterNode::calculateGyroMean, this);

  raw_angles_publisher = nh.advertise<std_msgs::Float64MultiArray>("rpy_raw", 1);
  filtered_angles_publisher = nh.advertise<std_msgs::Float64MultiArray>("rpy_filtered", 1);
  filtered_quaternion_publisher = nh.advertise<std_msgs::Float64MultiArray>("quaternion_filtered", 1);
  
  rpy_raw->data.resize(node_count * 3);
  rpy_filtered->data.resize(node_count * 3);
  quaternion_filtered->data.resize(node_count * 4);

  // Set up axes permutation
  // Axes are permuted so that each sensor in null-pose (standing, arms in front) gives zero angles 
  axes_permutation.resize(node_count);
  Eigen::Matrix3d upper_body(3,3), lower_back(3,3), left_thigh(3,3), left_calf(3,3), right_thigh(3,3), right_calf(3,3);
  upper_body  << 0,  0,  1,  0, -1,  0,  1,  0,  0;
  lower_back  << 0,  0, -1,  0,  1,  0,  1,  0,  0;
  left_thigh  << 0, -1,  0,  0,  0,  1, -1,  0,  0;
  left_calf   << 0,  0,  1,  0,  1,  0, -1,  0,  0;
  right_thigh << 0,  1,  0,  0,  0, -1, -1,  0,  0;
  right_calf  << 0,  0,  1,  0,  1,  0, -1,  0,  0;
  axes_permutation[0] = Eigen::Matrix3d::Identity(3,3);
  axes_permutation[1] = Eigen::Matrix3d::Identity(3,3);
  axes_permutation[2] = Eigen::Matrix3d::Identity(3,3);
  axes_permutation[3] = Eigen::Matrix3d::Identity(3,3);
  axes_permutation[4] = Eigen::Matrix3d::Identity(3,3);
  axes_permutation[5] = Eigen::Matrix3d::Identity(3,3);
  axes_permutation[6] = Eigen::Matrix3d::Identity(3,3);
  axes_permutation[7] = Eigen::Matrix3d::Identity(3,3);
  axes_permutation[8] = Eigen::Matrix3d::Identity(3,3);
  axes_permutation[9] = upper_body.transpose();
  axes_permutation[10] = Eigen::Matrix3d::Identity(3,3);
  axes_permutation[11] = Eigen::Matrix3d::Identity(3,3);
  axes_permutation[12] = left_calf.transpose();
  axes_permutation[13] = left_thigh.transpose();
  axes_permutation[14] = lower_back.transpose();
  axes_permutation[15] = Eigen::Matrix3d::Identity(3,3);
  axes_permutation[16] = right_thigh.transpose();
  axes_permutation[17] = right_calf.transpose();
  axes_permutation[18] = Eigen::Matrix3d::Identity(3,3);
  axes_permutation[19] = Eigen::Matrix3d::Identity(3,3);

  gyro_bias = Eigen::MatrixXd::Zero(node_count, 3);
  raw_data = nh.subscribe("net_data", 1, &DiverNetFilterNode::processData, this);
}

DiverNetFilterNode::~DiverNetFilterNode() {}

void DiverNetFilterNode::setCalibrationRequest(const std_msgs::Bool::ConstPtr& calibrate) {
  if (calibrate->data) {
    should_calibrate = true;
  }
}

void DiverNetFilterNode::calculateGyroMean(
    const std_msgs::Bool::ConstPtr& calculate_gyro_mean) {
  if (calculate_gyro_mean->data) {
    ROS_WARN("Gyro mean calculation in progress. Keep the nodes as still as possible.");
    gyro_bias = Eigen::MatrixXd::Zero(20,3);
    gyro_mean_calculation_frames_left = gyro_mean_num_frames;
  }
}

// Calibration procedure. The axes of each sensor is independently permuted
// so that, during the calibration, they match the calibration pose.
void DiverNetFilterNode::calibratePose(const std::vector<Eigen::Quaternion<double> >& q) {
  // Use head as reference for yaw orientation
  Eigen::Quaternion<double> head_yaw_quaternion;
  double r, p, y;
  labust::tools::eulerZYXFromQuaternion<Eigen::Quaternion<double> > (q[3], r, p, y);
  labust::tools::quaternionFromEulerZYX(0, 0, y, head_yaw_quaternion);

  // Referent pose - arms in front
  Eigen::MatrixXd referent_pose = Eigen::MatrixXd::Zero(node_count, 3);
  for (int i=0; i<node_count; ++i) {
    Eigen::Quaternion<double> pose_quaternion;
    labust::tools::quaternionFromEulerZYX(referent_pose(i,0), 
                                          referent_pose(i,1),
                                          referent_pose(i,2),
                                          pose_quaternion);
    pose_quaternion = head_yaw_quaternion * pose_quaternion;
    Eigen::Quaternion<double> offset = q[i].inverse() * pose_quaternion;
    Eigen::Quaternion<double> permutation(axes_permutation[i]);
    permutation = permutation * offset;
    axes_permutation[i] = permutation.matrix();
  }
}

void DiverNetFilterNode::processData(const std_msgs::Int16MultiArrayPtr &raw_data) {
  const int elem_count(9);
    
  // Normalize raw data
  Eigen::MatrixXd raw(node_count,elem_count);
  for (int i=0; i<node_count; ++i) {
    for (int e=0; e<elem_count; ++e) {
      raw(i,e) = raw_data->data[i*elem_count + e];
      raw(i,e) /= (1<<15);
    }
    // Normalize gyro data to rad/s
    raw(i,6) *= 2000.0 * M_PI/180;
    raw(i,7) *= 2000.0 * M_PI/180;
    raw(i,8) *= 2000.0 * M_PI/180;

    // Compensate gyro bias if mean calculation is not in progress
    if (gyro_mean_calculation_frames_left == 0) {
      raw(i,6) -= gyro_bias(i,0);
      raw(i,7) -= gyro_bias(i,1);
      raw(i,8) -= gyro_bias(i,2);
    } else {
      gyro_bias(i,0) += raw(i,6);
      gyro_bias(i,1) += raw(i,7);
      gyro_bias(i,2) += raw(i,8);
    
    }

    // Perform axes permutation to match the diver model and calibrated node positions.
    raw.block<1,3>(i,0) = raw.block<1,3>(i,0) * axes_permutation[i];
    raw.block<1,3>(i,3) = raw.block<1,3>(i,3) * axes_permutation[i];
    raw.block<1,3>(i,6) = raw.block<1,3>(i,6) * axes_permutation[i];
  }
  
  if (gyro_mean_calculation_frames_left > 0) {
    if (gyro_mean_calculation_frames_left == 1) {
      gyro_bias /= gyro_mean_num_frames;
      filter->reset();
      ROS_INFO("Gyro mean calculation complete.");
    }
    gyro_mean_calculation_frames_left--;
  }

  // Calculate raw Euler angles from accelerometer and magnetometer.  
  calculateRawAngles(raw);

  // Process each sensor with a filter to obtain filtered orientation.
  filter->processFrame(raw);
  std::vector<Eigen::Quaternion<double> > q = filter->getQuaternionOrientation();
  if (should_calibrate) {
    ROS_WARN("Pose calibration in progress.");
    calibratePose(q);
    should_calibrate = false;
    filter->reset();
    ROS_INFO("Pose calibration done.");
  }
  for (int i=0; i<node_count; ++i) {
    const double q0 = q[i].w();
    const double q1 = q[i].x();
    const double q2 = q[i].y();
    const double q3 = q[i].z();
    
    rpy_filtered->data[3*i] = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
    rpy_filtered->data[3*i+1] = asin(2.0 * (q0 * q2 - q3 * q1));
    rpy_filtered->data[3*i+2] = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
    
    quaternion_filtered->data[4*i] = q0;
    quaternion_filtered->data[4*i+1] = q1;
    quaternion_filtered->data[4*i+2] = q2;
    quaternion_filtered->data[4*i+3] = q3;
  }

  // raw_angles_publisher.publish(rpy_raw);
  filtered_angles_publisher.publish(rpy_filtered);
  filtered_quaternion_publisher.publish(quaternion_filtered);
}

void DiverNetFilterNode::calculateRawAngles(const Eigen::MatrixXd raw) {
  enum {ax,ay,az,mx,my,mz,gx,gy,gz};
  for (int i=0; i<node_count; ++i) {
    double roll = atan2(raw(i,ay), raw(i,az));
    double pitch = -atan2(raw(i,ax), sqrt(raw(i,ay) * raw(i,ay) + raw(i,az) * raw(i,az)));
    double hx = raw(i,mx) * cos(pitch) + 
                raw(i,my) * sin(pitch) * sin(roll) + 
                raw(i,mz) * cos(roll) * sin(pitch);
    double hy = raw(i,my) * cos(roll) - raw(i,mz) * sin(roll);
    double yaw = -atan2(-hy, hx);

    rpy_raw->data[3*i] = roll;
    rpy_raw->data[3*i+1] = pitch;
    rpy_raw->data[3*i+2] = yaw;
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "diver_net_filter_node");
  DiverNetFilterNode node;
  ros::spin();
  return 0;
}
