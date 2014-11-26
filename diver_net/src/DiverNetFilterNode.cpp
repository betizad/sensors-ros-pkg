#include <labust/sensors/DiverNetFilterNode.hpp>
#include <labust/sensors/ImuComplementaryQuaternionFilter.hpp>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

using namespace labust::sensors;

DiverNetFilterNode::DiverNetFilterNode():
    fs(50.0),
    dT(1.0/50),
    node_count(20),
    data_per_node(18),
    rpy_raw(new std_msgs::Float64MultiArray()),
    rpy_filtered(new std_msgs::Float64MultiArray()), 
    filter(new ImuComplementaryQuaternionFilter(20, 1.0/50, 0.18, 2000.0 * M_PI/180)) {
  this->onInit();
}

void DiverNetFilterNode::onInit() {
  ros::NodeHandle nh;
  ros::Rate rate(1);

  raw_angles_publisher = nh.advertise<std_msgs::Float64MultiArray>("rpy_raw", 1);
  filtered_angles_publisher = nh.advertise<std_msgs::Float64MultiArray>("rpy_filtered", 1);
  
  rpy_raw->data.resize(node_count * 3);
  rpy_filtered->data.resize(node_count * 3);

  raw_data = nh.subscribe("net_data", 1, &DiverNetFilterNode::processData, this);
}

DiverNetFilterNode::~DiverNetFilterNode() {}

void DiverNetFilterNode::processData(const std_msgs::Int16MultiArrayPtr &raw_data) {
  const int elem_count(9);

  // Rearange raw data for convenience
  Eigen::MatrixXd raw(node_count,elem_count);
  for (int i=0; i<node_count; ++i) {
    for (int e=0; e<elem_count; ++e) {
      raw(i,e) = raw_data->data[i*elem_count + e];
      raw(i,e) /= (1<<15);
    }
  }  

  // Calculate raw Euler angles from accelerometer and magnetometer.  
  calculateRawAngles(raw);

  // Process each sensor with a filter to obtain filtered orientation.
  filter->processFrame(raw);
  std::vector<Eigen::Quaternion<double> > q = filter->getQuaternionOrientation();
  for (int i=0; i<node_count; ++i) {
    double q0 = q[i].w();
    double q1 = q[i].x();
    double q2 = q[i].y();
    double q3 = q[i].z();
    
    rpy_filtered->data[3*i] = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
    rpy_filtered->data[3*i+1] = asin(2.0 * (q0 * q2 - q3 * q1));
    rpy_filtered->data[3*i+2] = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
  }

  raw_angles_publisher.publish(rpy_raw);
  filtered_angles_publisher.publish(rpy_filtered);
}

void DiverNetFilterNode::calculateRawAngles(const Eigen::MatrixXd raw) {
  enum {ax,ay,az,mx,my,mz,gx,gy,gz};
  for (int i=0; i<node_count; ++i) {
    double roll = atan2(raw(i,ay), raw(i,az));
    double pitch = -atan2(raw(i,ax), sqrt(raw(i,ay) * raw(i,ay) + raw(i,az) * raw(i,az)));
    double hx = raw(i,mx) * cos(pitch) + raw(i,my) * sin(pitch) * sin(roll) + raw(i,mz) * cos(roll) * sin(pitch);
    double hy = raw(i,my) * cos(roll) - raw(i,mz) * sin(roll);
    double yaw = atan2(-hy, hx);

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
