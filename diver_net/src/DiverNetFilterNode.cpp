#include <labust/sensors/DiverNetFilterNode.h>
#include <labust/sensors/DspUtils.hpp>
#include <labust/tools/conversions.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <fstream>

using namespace labust::sensors;

DiverNetFilterNode::DiverNetFilterNode():
    ph_("~"),
    node_count_(20),
    gyro_bias_(20,3),
    gyro_mean_calculation_frames_left_(0),
    gyro_mean_num_frames_(100),
    mag_cal_status_(None),
    pose_cal_status_(None),
    gyro_mean_status_(None),
    rpy_raw_(new std_msgs::Float64MultiArray()),
    rpy_filtered_(new std_msgs::Float64MultiArray()), 
    quaternion_filtered_(new std_msgs::Float64MultiArray()) {
  this->onInit();
}

void DiverNetFilterNode::onInit() {
  ros::Rate rate(1);

  pose_cal_sub_ = nh_.subscribe<std_msgs::Int16>(
      "calibrate_pose", 1, &DiverNetFilterNode::setPoseCalibrationRequest, this);
  gyro_mean_sub_ = nh_.subscribe<std_msgs::Int16>(
      "calculate_gyro_mean", 1, &DiverNetFilterNode::calculateGyroMean, this);
  mag_cal_sub_ = nh_.subscribe<std_msgs::Int16>(
      "calibrate_magnetometer", 1, &DiverNetFilterNode::setMagnetometerCalibrationRequest, this);

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
  pose_cal_permutation_.resize(node_count_);
  for (int i=0; i<node_count_; ++i) {
    pose_cal_permutation_[i] = Eigen::MatrixXd::Identity(3,3);
  }

  // Initialize Madgwick's filters
  filters_.resize(node_count_);
  for (int i=0; i<node_count_; ++i) {
    filters_[i].setAlgorithmGain(0.1);
    filters_[i].setDriftBiasGain(0.03);
  }

  raw_data_ = nh_.subscribe("net_data", 1, &DiverNetFilterNode::processData, this);

  // Motion rate buffers
  for (int i=0; i<node_count_; ++i) {
    for (int j=0; j<9; ++j) {
      data_buffer_[i][j].resize(2,0);
      hp_data_buffer_[i][j].resize(2,0);
    }
    for (int j=0; j<3; ++j) {
      hp_magnitude_buffer_[i][j].resize(2,0);
      hp_lp_magnitude_buffer_[i][j].resize(2,0);
    }
  }

  // Magnetometer buffer for calibration
  magnetometer_buffer_.resize(node_count_);
  magnetometer_buffer_points_ = 0;
  magnetometer_calibration_data_.resize(node_count_);
  for (int i=0; i<node_count_; ++i) {
    magnetometer_buffer_[i] = Eigen::MatrixXd::Zero(10000, 3);
  }

}

DiverNetFilterNode::~DiverNetFilterNode() {}

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
      for (int i=0; i<node_count_; ++i) {
        for (int j=0; j<3; ++j) {
          ifs >> gyro_bias_(i,j);
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
        Eigen::Vector3d center;
        for (int j=0; j<3; ++j) {
          ifs >> center(j);
        }
        Eigen::Matrix3d calib_matrix;
        for (int j=0; j<9; ++j) {
          ifs >> calib_matrix(j/3, j%3);
        }
        magnetometer_calibration_data_[i].center = center;
        magnetometer_calibration_data_[i].calib_matrix = calib_matrix;
      }
      ifs.close();
      ROS_INFO("Magnetometer calibration file opened successfully.");
      enable_mag_cal_ = true;
    }
  }  
}

void DiverNetFilterNode::setPoseCalibrationRequest(const std_msgs::Int16::ConstPtr& pose_cal) {
  if (pose_cal->data == 1 || (pose_cal->data == 2 && pose_cal_status_ == None)) {
    pose_cal_status_ = Pending;
  } else if (pose_cal->data == 2 && pose_cal_status_ == Off) {
    pose_cal_status_ = On;
    for (int i=0; i<node_count_; ++i) {
      filters_[i].reset();
    }
  } else if (pose_cal->data == 0) {
    pose_cal_status_ = Off;
    for (int i=0; i<node_count_; ++i) {
      filters_[i].reset();
    }
  }
}

void DiverNetFilterNode::calculateGyroMean(
    const std_msgs::Int16::ConstPtr& calculate_gyro_mean) {
  if (calculate_gyro_mean->data == 1 || (calculate_gyro_mean->data == 2 && gyro_mean_status_ == None)) {
    ROS_WARN("Gyro mean calculation in progress. Keep the nodes as still as possible.");
    gyro_bias_ = Eigen::MatrixXd::Zero(20,3);
    gyro_mean_calculation_frames_left_ = gyro_mean_num_frames_;
    gyro_mean_status_ = Pending;
  } else if (calculate_gyro_mean->data == 2 && gyro_mean_status_ == Off) {
    gyro_mean_status_ = On;
  } else if (calculate_gyro_mean->data == 0) {
    gyro_mean_status_ = Off;
  }
}

void DiverNetFilterNode::setMagnetometerCalibrationRequest(const std_msgs::Int16::ConstPtr& mag_cal) {
  if (mag_cal->data == 1 || (mag_cal->data == 2 && mag_cal_status_ == None)) {
    mag_cal_status_ = Pending;
    calculateMagnetometerCalibration();
    mag_cal_status_ = On;
  } else if (mag_cal->data == 2 && mag_cal_status_ == Off) {
    mag_cal_status_ = On;
  } else if (mag_cal->data == 0) {
    mag_cal_status_ = Off;
  }
}

void DiverNetFilterNode::calculateMagnetometerCalibration() {
  int rows = std::min(magnetometer_buffer_points_, 
      static_cast<int>(magnetometer_buffer_[0].rows()));
  MagnetometerCalibrationData mcd;
  for (int i=0; i<node_count_; ++i) {
    if (i==10 || i==15 || i==19) {
      mcd.center = Eigen::MatrixXd::Zero(3,1);
      mcd.calib_matrix = Eigen::MatrixXd::Identity(3,3);
      magnetometer_calibration_data_[i] = mcd;
      continue;
    }
    mcd = getMagnetometerCalibrationData(magnetometer_buffer_[i].block(0, 0, rows, 3));
    
    // Test if the calibration was successful
    if (mcd.sphere_coverage < 0.25 || mcd.avg_sqerror > 0.1) {
      ROS_WARN("Calibration for node %d unsuccessful, sphere coverage = %f, average square error = %f.", 
          i, mcd.sphere_coverage, mcd.avg_sqerror);
      mcd.center = Eigen::MatrixXd::Zero(3,1);
      mcd.calib_matrix = Eigen::MatrixXd::Identity(3,3);
    } else {
      ROS_INFO("Calibration for node %d completed successfully, sphere coverage = %f, average square error = %f.", 
          i, mcd.sphere_coverage, mcd.avg_sqerror);
    }
    magnetometer_calibration_data_[i] = mcd;
  }
  for (int i=0; i<node_count_; ++i) {
    std::cerr << magnetometer_calibration_data_[i].center << std::endl << magnetometer_calibration_data_[i].calib_matrix << std::endl;
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
    //axes_permutation_[i] = permutation.matrix();
    pose_cal_permutation_[i] = offset.matrix();
  }
}

void DiverNetFilterNode::calculateMotionRate() {
  std::vector<double> b_hp{0.9695, -0.9695}, a_hp{1.0, -0.9391};
  std::vector<double> b_lp{0.05}, a_lp{1.0, -0.95};
  for (int i=0; i<node_count_; ++i) {
    std::vector<double> mb(3,0);
    for (int j=0; j<9; ++j) {
      // High pass filter.
      hp_data_buffer_[i][j].push_back(
          filter(b_hp, a_hp, 
            data_buffer_[i][j].rbegin(), 
            hp_data_buffer_[i][j].rbegin()));
      mb[j/3] += *hp_data_buffer_[i][j].rbegin() * *hp_data_buffer_[i][j].rbegin(); 
    }
    for (int s=0; s<3; ++s) {
      // Calculate magnitudes and low pass filter them.
      hp_magnitude_buffer_[i][s].push_back(std::sqrt(mb[s]));
      hp_lp_magnitude_buffer_[i][s].push_back(
          filter(b_lp, a_lp, 
            hp_magnitude_buffer_[i][s].rbegin(), 
            hp_lp_magnitude_buffer_[i][s].rbegin()));
    }
  }

  // Sensors used in motion rate - head and limbs
  std::vector<int> mrs{3,4,8,11,18};
  double motion_rate = 0;
  for (int i=0; i<mrs.size(); ++i) {
    motion_rate = std::max(motion_rate, 
        *hp_lp_magnitude_buffer_[mrs[i]][0].rbegin() + *hp_lp_magnitude_buffer_[mrs[i]][2].rbegin());
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
      // Normalize acceleration data to g
      if (e/3 == 0) {
        raw(i,e) *= 16;
      }
      // Normalize magnetometer data to mT
      if (e/3 == 1) {
        raw(i,e) *= 1.2;
      }
      // Normalize gyro data to rad/s
      if (e/3 == 2) {
        raw(i,e) *= 2000.0 * M_PI/180;
      }

      // Filling data buffer. Currently the same data is in raw and in data buffer;
      // raw should be removed.
      data_buffer_[i][e].push_back(raw(i,e));
    }

    // Store magnetomter data for calibration
    int index = magnetometer_buffer_points_ % magnetometer_buffer_[i].rows();
    magnetometer_buffer_[i].row(index) = raw.block<1,3>(i,3);

    // Compensate gyro bias if mean calculation is not in progress
    if (gyro_mean_status_ == On) {
      raw(i,6) -= gyro_bias_(i,0);
      raw(i,7) -= gyro_bias_(i,1);
      raw(i,8) -= gyro_bias_(i,2);
    } else if (gyro_mean_status_ == Pending) {
      gyro_bias_(i,0) += raw(i,6);
      gyro_bias_(i,1) += raw(i,7);
      gyro_bias_(i,2) += raw(i,8);
    }

    // Perform axes permutation to match the diver model and calibrated node positions.
    raw.block<1,3>(i,0) = raw.block<1,3>(i,0) * axes_permutation_[i];
    raw.block<1,3>(i,3) = raw.block<1,3>(i,3) * axes_permutation_[i];
    raw.block<1,3>(i,6) = raw.block<1,3>(i,6) * axes_permutation_[i];
    if (pose_cal_status_ == On) {
      raw.block<1,3>(i,0) = raw.block<1,3>(i,0) * pose_cal_permutation_[i];
      raw.block<1,3>(i,3) = raw.block<1,3>(i,3) * pose_cal_permutation_[i];
      raw.block<1,3>(i,6) = raw.block<1,3>(i,6) * pose_cal_permutation_[i];
    }
  }

  // Motion rate
  calculateMotionRate();

  // Magnetometer calibration
  if (mag_cal_status_ == On) {
    for (int i=0; i<node_count_; ++i) {
      raw.block<1,3>(i,3) = calibrateMagnetometer(
          raw.block<1,3>(i,3),
          magnetometer_calibration_data_[i]);
    }
  }
  magnetometer_buffer_points_++;
  
  if (gyro_mean_calculation_frames_left_ > 0) {
    if (gyro_mean_calculation_frames_left_ == 1) {
      gyro_bias_ /= gyro_mean_num_frames_;
      for (int i=0; i<node_count_; ++i) {
        filters_[i].reset();
        double zeta = (gyro_bias_(i,0) + gyro_bias_(i,1) + gyro_bias_(i,2))/3;
        //std::cerr << zeta << std::endl;
        //filters_[i].setDriftBiasGain(-zeta);
      }
      std::cerr << gyro_bias_ << std::endl;
      ROS_INFO("Gyro mean calculation complete.");
      gyro_mean_status_ = On;
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
  if (pose_cal_status_ == Pending) {
    ROS_WARN("Pose calibration in progress.");
    calibratePose(q);
    for (int i=0; i<node_count_; ++i) {
      filters_[i].reset();
    }
    ROS_INFO("Pose calibration done.");
    pose_cal_status_ = On;
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
  ros::spin();
  return 0;
}
