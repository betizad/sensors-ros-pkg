#include <labust/sensors/DiverNetTransformAndJointPublisher.h>
#include <labust/tools/conversions.hpp>

#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Dense>

#include <boost/bind.hpp>
#include <boost/crc.hpp>

#include <fstream>
#include <iosfwd>

using namespace labust::sensors; 

DiverNetTransformAndJointPublisher::DiverNetTransformAndJointPublisher():
    node_count_(20),
    ph_("~"),
    joint_constraints_zyx_(20, 6),
    constrain_joints_(false),
    listener(tfbuffer) {
    this->onInit(); 
}

DiverNetTransformAndJointPublisher::~DiverNetTransformAndJointPublisher() {} 

void DiverNetTransformAndJointPublisher::onInit() {
  ros::Rate r(1);

  //Setup publisher
  jointsPub = nh_.advertise<sensor_msgs::JointState>("joint_states",1);
  quaternion_sub = nh_.subscribe("rpy_filtered", 1, 
      &DiverNetTransformAndJointPublisher::publishTransformAndJoints, this);

  configureNet();
  loadJointConstraints();
}

void DiverNetTransformAndJointPublisher::loadJointConstraints() {
  std::string joint_constraints_file;
  ph_.getParam("joint_constraints_file", joint_constraints_file);
  std::ifstream ifs;
  ifs.open(joint_constraints_file.c_str());
  if (!ifs.good()) {
    ROS_ERROR("No joint constraints file!");
  } else {
    for (int i=0; i<node_count_; ++i) {
      for (int j=0; j<6; ++j) {
        ifs >> joint_constraints_zyx_(i,j);
      }
    }
  }
}

void DiverNetTransformAndJointPublisher::configureNet() {
  //Connect imu number with joint
  //Should be a configuration option or from a urdf file
  names.push_back("right_shoulder");
  names.push_back("right_upper_arm");
  names.push_back("right_forearm");
  names.push_back("head"); //Inverter - head is number 4 and right hand is number 5 actually
  names.push_back("right_hand");
  names.push_back("left_shoulder");
  names.push_back("left_upper_arm");
  names.push_back("left_forearm");
  names.push_back("left_hand");
  names.push_back("upper_body");
  names.push_back("placeholder_1");
  names.push_back("left_foot");
  names.push_back("left_calf");
  names.push_back("left_thigh");
  names.push_back("lower_back");
  names.push_back("placeholder_2");
  names.push_back("right_thigh");
  names.push_back("right_calf");
  names.push_back("right_foot");
  names.push_back("placeholder_3");
}

void DiverNetTransformAndJointPublisher::publishTransformAndJoints(const std_msgs::Float64MultiArrayPtr &rpy) {
	enum {ax,ay,az,mx,my,mz,gx,gy,gz};
  Eigen::Quaternion<double> q;
  
  sensor_msgs::JointStatePtr joints(new sensor_msgs::JointState());
  joints->name.resize(node_count_ * 3);
  joints->position.resize(node_count_ * 3);
  joints->velocity.resize(node_count_ * 3);
  joints->effort.resize(node_count_ * 3);

	std::vector<geometry_msgs::TransformStamped> transforms(node_count_);

  for (int i=0; i<node_count_; ++i) {
    transforms[i].transform.translation.x = 0;
    transforms[i].transform.translation.y = 0;
    transforms[i].transform.translation.z = 0;
    labust::tools::quaternionFromEulerZYX(
      rpy->data[3*i], 
      rpy->data[3*i+1],
      rpy->data[3*i+2],
      q);
    
    transforms[i].transform.rotation.w = q.w();
    transforms[i].transform.rotation.x = q.x();
    transforms[i].transform.rotation.y = q.y();
    transforms[i].transform.rotation.z = q.z(); 

    /*transforms[i].transform.rotation.w = quaternion->data[4*i];
    transforms[i].transform.rotation.x = quaternion->data[4*i+1];
    transforms[i].transform.rotation.y = quaternion->data[4*i+2];
    transforms[i].transform.rotation.z = quaternion->data[4*i+3];*/
    
    transforms[i].child_frame_id = "abs_" + names[i];
    transforms[i].header.frame_id = "local";
    transforms[i].header.stamp = ros::Time::now(); 
  }
	
  broadcast.sendTransform(transforms);

  for (int i=0; i<node_count_; ++i) {
    if (i<20) {
      std::string parent;
      double roll(0),pitch(0),yaw(0);
      if (tfbuffer._getParent(names[i]+"_2", ros::Time(0), parent)) {
        geometry_msgs::TransformStamped trans;
        if (std::find(names.begin(), names.end(), parent) != names.end()) {
           if (tfbuffer.canTransform("abs_"+parent, "abs_"+names[i], ros::Time(0))) {
             trans = tfbuffer.lookupTransform("abs_"+parent, "abs_"+names[i], ros::Time(0));
           }
        } else {
          if (tfbuffer.canTransform("local", "abs_"+names[i], ros::Time(0))) {
            trans = tfbuffer.lookupTransform("local", "abs_"+names[i], ros::Time(0));
          }
        }

        labust::tools::eulerZYXFromQuaternion(trans.transform.rotation, roll, pitch, yaw);
      
        /*std::cerr << "YAW: " << yaw*180/M_PI << "\nPITCH: " << pitch*180/M_PI << "\nROLL: " << roll*180/M_PI << "\n";
        
        if (abs(yaw) > M_PI/2 && abs(roll) > M_PI/2) {
          if (yaw < 0) yaw += M_PI;
          else if (yaw > 0) yaw -= M_PI;
          if (roll < 0) roll += M_PI;
          else if (roll > 0) roll -= M_PI;
          if (pitch < 0) pitch = -M_PI - pitch;
          else if (pitch > 0) pitch = M_PI - pitch;
        }
        std::cerr << "YAW: " << yaw*180/M_PI << "\nPITCH: " << pitch*180/M_PI << "\nROLL: " << roll*180/M_PI << "\n";
        
        if (yaw < joint_constraints_zyx_(i,0) || yaw > joint_constraints_zyx_(i,1)) {
          double dist_0 = std::min(abs(yaw - joint_constraints_zyx_(i,0)), abs(yaw-2*M_PI - joint_constraints_zyx_(i,0)));
          double dist_1 = std::min(abs(yaw - joint_constraints_zyx_(i,1)), abs(yaw+2*M_PI - joint_constraints_zyx_(i,1)));
          if (dist_0 < dist_1) yaw = joint_constraints_zyx_(i,0);
          else yaw = joint_constraints_zyx_(i,1);
        }

        if (pitch < joint_constraints_zyx_(i,2) || pitch > joint_constraints_zyx_(i,3)) {
          double dist_0 = std::min(abs(pitch - joint_constraints_zyx_(i,2)), abs(pitch-2*M_PI - joint_constraints_zyx_(i,2)));
          double dist_1 = std::min(abs(pitch - joint_constraints_zyx_(i,3)), abs(pitch+2*M_PI - joint_constraints_zyx_(i,3)));
          if (dist_0 < dist_1) pitch = joint_constraints_zyx_(i,2);
          else pitch = joint_constraints_zyx_(i,3);
        }
        
        if (roll < joint_constraints_zyx_(i,4) || roll > joint_constraints_zyx_(i,5)) {
          double dist_0 = std::min(abs(roll - joint_constraints_zyx_(i,0)), abs(roll-2*M_PI - joint_constraints_zyx_(i,4)));
          double dist_1 = std::min(abs(roll - joint_constraints_zyx_(i,1)), abs(roll+2*M_PI - joint_constraints_zyx_(i,5)));
          if (dist_0 < dist_1) roll = joint_constraints_zyx_(i,4);
          else roll = joint_constraints_zyx_(i,5);
        }


        std::cerr << "YAW: " << yaw*180/M_PI << "\nPITCH: " << pitch*180/M_PI << "\nROLL: " << roll*180/M_PI << "\n\n";
       */
      } 

      joints->name[3*i] = names[i] + "_x";
      joints->name[3*i+1] = names[i] + "_y";
      joints->name[3*i+2] = names[i] + "_z";
      boost::mutex::scoped_lock l(dataMux); 
      joints->position[3*i] = roll; 
      joints->position[3*i+1] = pitch;
      joints->position[3*i+2] = yaw; 
      l.unlock();
    } else {
      joints->name[3*i] = names[i] + "_x";
      joints->name[3*i+1] = names[i] + "_y";
      joints->name[3*i+2] = names[i] + "_z";
      joints->position[3*i] = 0;
      joints->position[3*i+1] = 0;
      joints->position[3*i+2] = 0;
    }
  }

  //Process joints
  //Stamp data
  joints->header.stamp = ros::Time::now();
  //Publish data
  jointsPub.publish(joints);
}

int main(int argc, char* argv[]) {
	ros::init(argc,argv,"diver_net_transform_and_joint_publisher_node");
	DiverNetTransformAndJointPublisher node; 
	ros::spin();
	return 0;
}
