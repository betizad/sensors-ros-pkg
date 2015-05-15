#include <labust/sensors/DiverNetGyroCalibrationNode.hpp>
#include <labust/sensors/ImuComplementaryQuaternionFilter.hpp>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <fstream>

using namespace labust::sensors;


DiverNetGyroCalibrationNode::DiverNetGyroCalibrationNode():
    n(0),
    max_frames(500),
    elem_count(9),
    node_count(20),
    calibration_file_("diver_net_gyro_bias") {
  ros::NodeHandle nh;
  gyro_bias = Eigen::MatrixXd::Zero(20,3); 
  raw_data = nh.subscribe("net_data", 1, &DiverNetGyroCalibrationNode::processData, this);
}

DiverNetGyroCalibrationNode::~DiverNetGyroCalibrationNode() {}

void DiverNetGyroCalibrationNode::processData(const std_msgs::Int16MultiArrayPtr &raw_data) {
  // Normalize raw data
  Eigen::MatrixXd raw(node_count,elem_count);
  for (int i=0; i<node_count; ++i) {
    for (int e=6; e<elem_count; ++e) {
      raw(i,e-6) = raw_data->data[i*elem_count + e]; 
      raw(i,e-6) /= (1<<15);
    }   
    // Normalize gyro data to rad/s
    raw(i,0) *= 2000.0 * M_PI/180;
    raw(i,1) *= 2000.0 * M_PI/180;
    raw(i,2) *= 2000.0 * M_PI/180;

    gyro_bias(i,0) += raw(i,0);
    gyro_bias(i,1) += raw(i,1);
    gyro_bias(i,2) += raw(i,2);
  }

  n++;
  if (n == max_frames) {
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer,80,"_%Y-%m-%d-%H-%M-%S.dnc",timeinfo);
    std::string str(buffer);
    std::ofstream ofs;
    ofs.open((calibration_file_+str).c_str(), std::ofstream::out);
    ofs << gyro_bias / max_frames; 
    ofs.close();
    ros::shutdown(); 
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "diver_net_gyro_calibration_node");
  DiverNetGyroCalibrationNode node;
  ros::spin();
  return 0;
}
