#include <labust/sensors/DiverNetFilterNode.h>
#include <labust/tools/conversions.hpp>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <labust/sensors/ImuFilter.h>
#include <fstream>

using namespace labust::sensors;

DiverNetFilterNode::DiverNetFilterNode():
    node_count_(20),
    ph_("~"),
    gyro_bias_(20,3),
    magnetometer_ellipsoid_center_(20,3),
    magnetometer_ellipsoid_scale_(20,3),
    should_calibrate_pose_(false),
    should_calibrate_magnetometer_(false),
    gyro_mean_calculation_frames_left_(0),
    gyro_mean_num_frames_(100),
    rpy_raw_(new std_msgs::Float64MultiArray()),
    rpy_filtered_(new std_msgs::Float64MultiArray()), 
    quaternion_filtered_(new std_msgs::Float64MultiArray()),
    filters_(20),
    data_buffer_(2),
    hp_filtered_buffer_(2),
    magnitude_buffer_(2),
    magnitude_filtered_(2) {
  this->onInit();
}

void DiverNetFilterNode::onInit() {
  ros::Rate rate(1);

  calibrate_sub_ = nh_.subscribe<std_msgs::Bool>(
      "calibrate", 1, &DiverNetFilterNode::setCalibrationRequest, this);
  gyro_mean_sub_ = nh_.subscribe<std_msgs::Bool>(
      "calculate_gyro_mean", 1, &DiverNetFilterNode::calculateGyroMean, this);

  raw_angles_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("rpy_raw", 1);
  filtered_angles_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("rpy_filtered", 1);
  filtered_quaternion_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("quaternion_filtered", 1);
  motion_rate_pub_ = nh_.advertise<std_msgs::Float64>("motion_rate", 1);

  rpy_raw_->data.resize(node_count_ * 3);
  rpy_filtered_->data.resize(node_count_ * 3);
  quaternion_filtered_->data.resize(node_count_ * 4);

  // Set up axes permutation
  // Axes are permuted so that each sensor in null-pose (standing, arms in front) gives zero angles 
  loadModelAxesPermutation();
  loadGyroBiasFromFile();
  loadMagnetometerCalibration();

  for (int i=0; i<node_count_; ++i) {
    filters_[i].setAlgorithmGain(0.1);
    filters_[i].setDriftBiasGain(0.03);
  }

  raw_data_ = nh_.subscribe("net_data", 1, &DiverNetFilterNode::processData, this);

  Eigen::MatrixXd z = Eigen::MatrixXd::Zero(20,9);
  Eigen::MatrixXd zz = Eigen::MatrixXd::Zero(20,3);
  for (int i = 0; i<2; ++i) {
    data_buffer_.push_back(z);
    hp_filtered_buffer_.push_back(z);
    magnitude_buffer_.push_back(zz);
    magnitude_filtered_.push_back(zz);
  }
}

void DiverNetFilterNode::loadModelAxesPermutation() {
  axes_permutation_.resize(node_count_);
  std::string model_axes_permutation_file;
  ph_.getParam("model_axes_permutation_file", model_axes_permutation_file);
  std::ifstream ifs;
  ifs.open(model_axes_permutation_file.c_str());
  if (!ifs.good()) {
    ROS_ERROR("No axes permutation file!");
  } else {
    for (int i=0; i<node_count_; ++i) {
      for (int j=0; j<9; ++j) {
        ifs >> axes_permutation_[i](j%3, j/3);
      }
    }  
  }
}

void DiverNetFilterNode::loadGyroBiasFromFile() {
  std::string calibration_file("");
  // Gyro sensor bias
  gyro_bias_ = Eigen::MatrixXd::Zero(node_count_, 3);
  ph_.getParam("gyro_mean_file", calibration_file);
  if (calibration_file == "") {
    ROS_ERROR("No gyro calibration file provided, zero mean offset assumed.");
  } else {
    ROS_ERROR("Opening gyro calibration file %s", calibration_file.c_str());
    std::ifstream ifs;
    ifs.open(calibration_file.c_str());
    if (!ifs.good()) {
      ROS_ERROR("File %s does not exist or cannot be opened.", calibration_file.c_str());
    } else {
      double gb;
      for (int i=0; i<node_count_; ++i) {
        for (int j=0; j<3; ++j) {
          ifs >> gb;
          gyro_bias_(i,j) = gb;
        }
      }
      ifs.close();
      ROS_INFO("Gyro mean file opened successfully.");
    }
  }  
}

void DiverNetFilterNode::loadMagnetometerCalibration() {
  std::string calibration_file = "";
  // Magnetometer calibration
  ph_.getParam("magnetometer_calibration_file", calibration_file);
  if (calibration_file == "") {
    ROS_ERROR("No magnetometer calibration file provided.");
  } else {
    ROS_ERROR("Opening magnetometer calibration file %s", calibration_file.c_str());
    std::ifstream ifs;
    ifs.open(calibration_file.c_str());
    if (!ifs.good()) {
      ROS_ERROR("File %s does not exist or cannot be opened.", calibration_file.c_str());
    } else {
      for (int i=0; i<node_count_; ++i) {
        for (int j=0; j<3; ++j) {
          ifs >> magnetometer_ellipsoid_center_(i,j);
        }
        for (int j=0; j<3; ++j) {
          ifs >> magnetometer_ellipsoid_scale_(i,j);
        }
      }
      ifs.close();
      ROS_INFO("Magnetometer calibration file opened successfully.");
      should_calibrate_magnetometer_ = true;
    }
  }  
}

DiverNetFilterNode::~DiverNetFilterNode() {}

void DiverNetFilterNode::setCalibrationRequest(const std_msgs::Bool::ConstPtr& calibrate) {
  if (calibrate->data) {
    should_calibrate_pose_ = true;
  }
}

void DiverNetFilterNode::calculateGyroMean(
    const std_msgs::Bool::ConstPtr& calculate_gyro_mean) {
  if (calculate_gyro_mean->data) {
    ROS_WARN("Gyro mean calculation in progress. Keep the nodes as still as possible.");
    gyro_bias_ = Eigen::MatrixXd::Zero(20,9);
    gyro_mean_calculation_frames_left_ = gyro_mean_num_frames_;
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
  Eigen::MatrixXd referent_pose = Eigen::MatrixXd::Zero(node_count_, 3);
  for (int i=0; i<node_count_; ++i) {
    Eigen::Quaternion<double> pose_quaternion;
    labust::tools::quaternionFromEulerZYX(referent_pose(i,0), 
                                          referent_pose(i,1),
                                          referent_pose(i,2),
                                          pose_quaternion);
    pose_quaternion = head_yaw_quaternion * pose_quaternion;
    Eigen::Quaternion<double> offset = q[i].inverse() * pose_quaternion;
    Eigen::Quaternion<double> permutation(axes_permutation_[i]);
    permutation = permutation * offset;
    axes_permutation_[i] = permutation.matrix();
  }
}

void DiverNetFilterNode::calculateMotionRate() {
  Eigen::MatrixXd z = Eigen::MatrixXd::Zero(20,9);
  Eigen::MatrixXd z_m = Eigen::MatrixXd::Zero(20,3);
  Eigen::MatrixXd z_m_filtered = Eigen::MatrixXd::Zero(20,3);
  std::vector<double> b_hp, a_hp;
  std::vector<double> b_lp, a_lp;
  b_hp.push_back(0.9695);
  b_hp.push_back(-0.9695);
  a_hp.push_back(0);
  a_hp.push_back(-0.9391);
  a_lp.push_back(0);
  a_lp.push_back(-0.95);
  b_lp.push_back(0.05);
  for (int i=0; i<node_count_; ++i) {
    for (int j=0; j<9; ++j) {
      for (int k=0; k<b_hp.size(); ++k) {
        z(i,j) += b_hp[k] * data_buffer_[data_buffer_.size()-1-k](i,j);
      }
      for (int k=1; k<a_hp.size(); ++k) {
        z(i,j) += -a_hp[k] * hp_filtered_buffer_[hp_filtered_buffer_.size()-k](i,j);
      }
      z_m(i,j/3) += z(i,j) * z(i,j);
    }
    for (int s=0; s<3; ++s) {
      z_m(i,s) = std::sqrt(z_m(i,s));
      z_m_filtered(i,s) = b_lp[0] * z_m(i,s) - a_lp[1] * magnitude_filtered_[magnitude_filtered_.size()-1](i,s); 
    }
  }
  hp_filtered_buffer_.push_back(z);
  magnitude_buffer_.push_back(z_m);
  magnitude_filtered_.push_back(z_m_filtered);

  std::vector<int> motion_rate_sensors;
  motion_rate_sensors.push_back(3);
  motion_rate_sensors.push_back(4);
  motion_rate_sensors.push_back(8);
  motion_rate_sensors.push_back(11);
  motion_rate_sensors.push_back(18);
  double motion_rate = 0;
  for (int i=0; i<motion_rate_sensors.size(); ++i) {
    motion_rate = std::max(motion_rate, magnitude_filtered_[magnitude_filtered_.size()-1](motion_rate_sensors[i],0) + magnitude_filtered_[magnitude_filtered_.size()-1](motion_rate_sensors[i],2));
  }
  std_msgs::Float64 mr;
  mr.data = motion_rate;
  motion_rate_pub_.publish(mr);
}

void DiverNetFilterNode::processData(const std_msgs::Int16MultiArrayPtr &raw_data) {
  const int elem_count(9);
    
  // Normalize raw data
  Eigen::MatrixXd raw(node_count_, elem_count);
  for (int i=0; i<node_count_; ++i) {
    for (int e=0; e<elem_count; ++e) {
      raw(i,e) = raw_data->data[i*elem_count + e];
      raw(i,e) /= (1<<15);
    }
    // Normalize acceleration data to g
    raw(i,0) *= 16;
    raw(i,1) *= 16;
    raw(i,2) *= 16;
    // Normalize gyro data to rad/s
    raw(i,6) *= 2000.0 * M_PI/180;
    raw(i,7) *= 2000.0 * M_PI/180;
    raw(i,8) *= 2000.0 * M_PI/180;

    // Calibrate magnetometer data
    if (should_calibrate_magnetometer_) {
      raw(i,3) = magnetometer_ellipsoid_scale_(i,0) * (raw(i,3) - magnetometer_ellipsoid_center_(i,0));
      raw(i,4) = magnetometer_ellipsoid_scale_(i,1) * (raw(i,4) - magnetometer_ellipsoid_center_(i,1));
      raw(i,5) = magnetometer_ellipsoid_scale_(i,2) * (raw(i,5) - magnetometer_ellipsoid_center_(i,2));
    }

    // Compensate gyro bias if mean calculation is not in progress
    if (gyro_mean_calculation_frames_left_ == 0) {
      raw(i,6) -= gyro_bias_(i,0);
      raw(i,7) -= gyro_bias_(i,1);
      raw(i,8) -= gyro_bias_(i,2);
    } else {
      gyro_bias_(i,0) += raw(i,6);
      gyro_bias_(i,1) += raw(i,7);
      gyro_bias_(i,2) += raw(i,8);
    }

    // Perform axes permutation to match the diver model and calibrated node positions.
    raw.block<1,3>(i,0) = raw.block<1,3>(i,0) * axes_permutation_[i];
    raw.block<1,3>(i,3) = raw.block<1,3>(i,3) * axes_permutation_[i];
    raw.block<1,3>(i,6) = raw.block<1,3>(i,6) * axes_permutation_[i];
  }

  // Motion rate
  data_buffer_.push_back(raw);
  calculateMotionRate();
  
  if (gyro_mean_calculation_frames_left_ > 0) {
    if (gyro_mean_calculation_frames_left_ == 1) {
      gyro_bias_ /= gyro_mean_num_frames_;
      for (int i=0; i<node_count_; ++i) {
        filters_[i].reset();
        double zeta = (gyro_bias_(i,0) + gyro_bias_(i,1) + gyro_bias_(i,2))/3;
        std::cerr << zeta << std::endl;
        //filters_[i].setDriftBiasGain(-zeta);
      }
      std::cerr << gyro_bias_ << std::endl;
      ROS_INFO("Gyro mean calculation complete.");
    }
    gyro_mean_calculation_frames_left_--;
  }

  // Calculate raw Euler angles from accelerometer and magnetometer.  
  calculateRawAngles(raw);

  // Process each sensor with a filter to obtain filtered orientation.
  std::vector<Eigen::Quaternion<double> > q(node_count_);
  for (int i=0; i<node_count_; ++i) {
    filters_[i].madgwickAHRSupdate(raw(i,6), raw(i,7), raw(i,8),
                                   raw(i,0), raw(i,1), raw(i,2),
                                   raw(i,3), raw(i,4), raw(i,5),
                                   1.0/10.0);
    filters_[i].getOrientation(q[i].w(), q[i].x(), q[i].y(), q[i].z());
  }
  if (should_calibrate_pose_) {
    ROS_WARN("Pose calibration in progress.");
    calibratePose(q);
    should_calibrate_pose_ = false;
    for (int i=0; i<node_count_; ++i) {
      filters_[i].reset();
    }
    ROS_INFO("Pose calibration done.");
  }
  for (int i=0; i<node_count_; ++i) {
    const double q0 = q[i].w();
    const double q1 = q[i].x();
    const double q2 = q[i].y();
    const double q3 = q[i].z();
    
    rpy_filtered_->data[3*i] = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
    rpy_filtered_->data[3*i+1] = asin(2.0 * (q0 * q2 - q3 * q1));
    rpy_filtered_->data[3*i+2] = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
    
    quaternion_filtered_->data[4*i] = q0;
    quaternion_filtered_->data[4*i+1] = q1;
    quaternion_filtered_->data[4*i+2] = q2;
    quaternion_filtered_->data[4*i+3] = q3;
  }

  // raw_angles_publisher.publish(rpy_raw);
  filtered_angles_pub_.publish(rpy_filtered_);
  filtered_quaternion_pub_.publish(quaternion_filtered_);
}

void DiverNetFilterNode::calculateRawAngles(const Eigen::MatrixXd raw) {
  enum {ax,ay,az,mx,my,mz,gx,gy,gz};
  for (int i=0; i<node_count_; ++i) {
    double roll = atan2(raw(i,ay), raw(i,az));
    double pitch = -atan2(raw(i,ax), sqrt(raw(i,ay) * raw(i,ay) + raw(i,az) * raw(i,az)));
    double hx = raw(i,mx) * cos(pitch) + 
                raw(i,my) * sin(pitch) * sin(roll) + 
                raw(i,mz) * cos(roll) * sin(pitch);
    double hy = raw(i,my) * cos(roll) - raw(i,mz) * sin(roll);
    double yaw = -atan2(-hy, hx);

    rpy_raw_->data[3*i] = roll;
    rpy_raw_->data[3*i+1] = pitch;
    rpy_raw_->data[3*i+2] = yaw;
  }
  raw_angles_pub_.publish(rpy_raw_);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "diver_net_filter_node");
  DiverNetFilterNode node;
  boost::circular_buffer<int> p(3);
  p.push_back(1);
  p.push_back(2);
  p.push_back(3);
  std::cerr << p[0] << " " << p[1] << " " << p[2] << std::endl;
  p.push_back(4);
  std::cerr << p[0] << " " << p[1] << " " << p[2] << std::endl;
  ros::spin();
  return 0;
}
