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
    is_initialized(false),
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

  filter->processFrame(raw);
  std::vector<std::vector<double> > q = filter->getQuaternionOrientation();
  for (int i=0; i<node_count; ++i) {
    double q0 = q[i][0];
    double q1 = q[i][1];
    double q2 = q[i][2];
    double q3 = q[i][3];
    
    rpy_filtered->data[3*i] = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
    rpy_filtered->data[3*i+1] = asin(2.0 * (q0 * q2 - q3 * q1));
    rpy_filtered->data[3*i+2] = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
  }

  calculateRawAngles(raw);
  //complementaryFilter(raw);
  
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

void DiverNetFilterNode::complementaryFilter(const Eigen::MatrixXd raw) { 
  enum {ax,ay,az,mx,my,mz,gx,gy,gz};

  if (!is_initialized) {
    for (int i=0; i<node_count; ++i) {
      rpy_filtered->data[3*i] = rpy_raw->data[3*i];
      rpy_filtered->data[3*i+1] = rpy_raw->data[3*i+1];
      rpy_filtered->data[3*i+2] = rpy_raw->data[3*i+2];
    }
    is_initialized = true;
    return;
  }
  // Complementary filter
  const double a = 0.01;
  const double gyro_gain = 2000.0 * M_PI / 180;
  for (int i=0; i<node_count; ++i) {
    double delta_roll = rpy_raw->data[3*i] - rpy_filtered->data[3*i];
    double delta_pitch = rpy_raw->data[3*i+1] - rpy_filtered->data[3*i+1];
    double delta_yaw = rpy_raw->data[3*i+2] - rpy_filtered->data[3*i+2];
    if (abs(delta_roll) > M_PI) rpy_raw->data[3*i] -= round(delta_roll/(2*M_PI)) * 2 * M_PI;
    if (abs(delta_pitch) > M_PI) rpy_raw->data[3*i+1] -= round(delta_pitch/(2*M_PI)) * 2 * M_PI;
    if (abs(delta_yaw) > M_PI) rpy_raw->data[3*i+2] -= round(delta_yaw/(2*M_PI)) * 2 * M_PI;

    double a1 = 0.03 - 0.03 * abs(rpy_raw->data[3*i] - rpy_filtered->data[3*i]) / (M_PI);
    double a2 = 0.03 - 0.03 * abs(rpy_raw->data[3*i+1] - rpy_filtered->data[3*i+1]) / (M_PI);
    double a3 = 0.03 - 0.03 * abs(rpy_raw->data[3*i+2] - rpy_filtered->data[3*i+2]) / (M_PI);
    if (abs(2*rpy_filtered->data[3*i+1]/M_PI) > 0.8) {
      a1 = 0.03;
      a2 = 0.03;
      a3 = 0.03;
    }
    if (abs(delta_roll) > M_PI/4) a1 = 0.1;
    if (abs(delta_pitch) > M_PI/4) a2 = 0.1;
    if (abs(delta_yaw) > M_PI/4) a3 = 0.1;
    double roll = (1-a1)*(rpy_filtered->data[3*i] + raw(i,gx)*dT*gyro_gain) + a1*rpy_raw->data[3*i];
    double pitch = (1-a2)*(rpy_filtered->data[3*i+1] + (cos(roll)*raw(i,gy) - sin(roll)*raw(i,gz))*dT*gyro_gain) + a2*rpy_raw->data[3*i+1];
    double yaw = (1-a3)*(rpy_filtered->data[3*i+2] + (-sin(pitch)*raw(i,gx) + cos(pitch)*sin(roll)*raw(i,gy) + cos(pitch)*cos(roll)*raw(i,gz)) * dT*gyro_gain) + a3*rpy_raw->data[3*i+2];
  
    rpy_filtered->data[3*i] = roll;
    rpy_filtered->data[3*i+1] = pitch;
    rpy_filtered->data[3*i+2] = yaw;
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "diver_net_filter_node");
  DiverNetFilterNode node;
  /*  ImuComplementaryQuaternionFilter filt(1, 1.f/200, 0.1);
  filt.printQuaternionOrientation();
  Eigen::MatrixXd data(1,9);
  data(0,0) = 0.3463;
  data(0,1) = 0.0742;
  data(0,2) = 10.5360;
  data(0,3) = -0.0026;
  data(0,4) =  0.0166;
  data(0,5) = -0.0013;
  data(0,6) = 26.7273;
  data(0,7) = 13.0909;
  data(0,8) = -36.1225;
  filt.processFrame(data);
  filt.printQuaternionOrientation();*/
  ros::spin();
  return 0;
}
