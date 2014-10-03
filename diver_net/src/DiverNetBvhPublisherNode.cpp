#include <labust/sensors/DiverNetBvhPublisherNode.hpp>
#include <sensor_msgs/JointState.h>
#include <fstream>


using namespace labust::sensors;


// TODO(irendulic): Add direct read from bag file. Track num_frames instead of requesting it.
DiverNetBvhPublisherNode::DiverNetBvhPublisherNode() {
  this->onInit();
}

void DiverNetBvhPublisherNode::onInit() {
  ros::NodeHandle nh, ph("~");
  ros::Rate rate(1);
  joint_states_subscriber = nh.subscribe("joint_states", 1, &DiverNetBvhPublisherNode::processData, this);
  ph.param("framerate", framerate, 0.1);
  ph.param("num_frames", num_frames, 0);
  ph.param<std::string>("bvh_model", bvh_model_file, "");
  std::ifstream bvh_file(bvh_model_file.c_str());
  std::string line;
  while (getline(bvh_file, line)) {
    std::cout << line << std::endl;
  }
  std::cout << "Frames: " << num_frames << std::endl;
  std::cout << "Frame Time: " << 1/framerate << std::endl;

  node_count = 17;
  joint_map.insert(std::pair<std::string, int>("lower_back", 0));
  joint_map.insert(std::pair<std::string, int>("upper_body", 1));
  joint_map.insert(std::pair<std::string, int>("head", 2));
  joint_map.insert(std::pair<std::string, int>("left_shoulder", 3));
  joint_map.insert(std::pair<std::string, int>("left_upper_arm", 4));
  joint_map.insert(std::pair<std::string, int>("left_forearm", 5));
  joint_map.insert(std::pair<std::string, int>("left_hand", 6));
  joint_map.insert(std::pair<std::string, int>("right_shoulder", 7));
  joint_map.insert(std::pair<std::string, int>("right_upper_arm", 8));
  joint_map.insert(std::pair<std::string, int>("right_forearm", 9));
  joint_map.insert(std::pair<std::string, int>("right_hand", 10));
  joint_map.insert(std::pair<std::string, int>("left_thigh", 11));
  joint_map.insert(std::pair<std::string, int>("left_calf", 12));
  joint_map.insert(std::pair<std::string, int>("left_foot", 13));
  joint_map.insert(std::pair<std::string, int>("right_thigh", 14));
  joint_map.insert(std::pair<std::string, int>("right_calf", 15));
  joint_map.insert(std::pair<std::string, int>("right_foot", 16));
}

DiverNetBvhPublisherNode::~DiverNetBvhPublisherNode() {}

void DiverNetBvhPublisherNode::processData(const sensor_msgs::JointState &joint_states) {
  std::vector<float> curr_joint_state(node_count * 3);
  for (int i=0; i<joint_states.name.size(); ++i) {
    std::string joint(joint_states.name[i].begin(), joint_states.name[i].end()-2);
    if (joint.find(std::string("place")) != std::string::npos) continue;
    std::string joint_axis(joint_states.name[i].end()-1, joint_states.name[i].end());
    int joint_number = 3*joint_map[joint];
    if (joint_axis == "x") joint_number +=2;
    if (joint_axis == "y") joint_number +=1;
    curr_joint_state[joint_number] = joint_states.position[i] * 180 / M_PI;
  }
  std::cout << "0 0 0 0 0 0 ";
  for (int i=0; i<curr_joint_state.size(); ++i) {
    std::cout << curr_joint_state[i] << " ";
  }
  std::cout << std::endl;
}


int main(int argc, char* argv[]) {
  ros::init(argc, argv, "diver_net_bvh_publisher_node");
  DiverNetBvhPublisherNode node;
  ros::spin();
  return 0;
}
