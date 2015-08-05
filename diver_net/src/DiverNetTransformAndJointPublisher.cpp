#include <labust/sensors/DiverNetTransformAndJointPublisher.hpp>
#include <labust/tools/conversions.hpp>

#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Dense>

#include <boost/bind.hpp>
#include <boost/crc.hpp>

#include <iosfwd>

using namespace labust::sensors; 

DiverNetTransformAndJointPublisher::DiverNetTransformAndJointPublisher():
    _rpy_topic("/rpy_filtered"),
    nodeCount(20),
    isCalibrated(false),
    listener(tfbuffer),
    zeroState(Eigen::MatrixXd(nodeCount, 3)),
    zeroStateQ(std::vector<Eigen::Quaternion<double> > (nodeCount)),
    currentMeasQ(std::vector<Eigen::Quaternion<double> > (nodeCount)),
    offsetQ(std::vector<Eigen::Quaternion<double> > (nodeCount)) {
    this->onInit(); 
}

DiverNetTransformAndJointPublisher::~DiverNetTransformAndJointPublisher() {} 

void DiverNetTransformAndJointPublisher::onInit() {
  ros::NodeHandle nh, ph("~"); 
  ros::Rate r(1);

  //Setup publisher
  jointsPub = nh.advertise<sensor_msgs::JointState>("joint_states",1);
  rpy = nh.subscribe(_rpy_topic, 1, &DiverNetTransformAndJointPublisher::publishTransformAndJoints, this);
  calibrate = nh.subscribe<std_msgs::Bool>("/calibrate", 1, &DiverNetTransformAndJointPublisher::calibratePose, this);

  configureNet();
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
  
  // Setup zero state - Should be a configuration option
  // Null-pose, arms in front. This initialization assumes axes permutation is done.
  zeroState = Eigen::MatrixXd::Zero(nodeCount, 3);
}


void DiverNetTransformAndJointPublisher::calibratePose(const std_msgs::Bool::ConstPtr& calibrate) {
	if (calibrate->data) {
    boost::mutex::scoped_lock l(dataMux);
    Eigen::Quaternion<double> q;
    double r, p, y;
    labust::tools::eulerZYXFromQuaternion<Eigen::Quaternion<double> > (currentMeasQ[3], r, p, y);
    labust::tools::quaternionFromEulerZYX(0, 0, y, q);
    for (int i=0; i<nodeCount; ++i) {
      labust::tools::quaternionFromEulerZYX(zeroState(i,0), zeroState(i,1), zeroState(i,2), zeroStateQ[i]);
      zeroStateQ[i] = q * zeroStateQ[i];
      offsetQ[i] = currentMeasQ[i].inverse() * zeroStateQ[i];
    }	
    isCalibrated = true;
	} else {
    isCalibrated = false;
  }
}

void DiverNetTransformAndJointPublisher::publishTransformAndJoints(const std_msgs::Float64MultiArrayPtr &rpy) {
	enum {ax,ay,az,mx,my,mz,gx,gy,gz};
 
  sensor_msgs::JointStatePtr joints(new sensor_msgs::JointState());
  joints->name.resize(nodeCount * 3);
  joints->position.resize(nodeCount * 3);
  joints->velocity.resize(nodeCount * 3);
  joints->effort.resize(nodeCount * 3);

	std::vector<geometry_msgs::TransformStamped> transforms(nodeCount);

  for (int i=0; i<nodeCount; ++i) {
    transforms[i].transform.translation.x = 0;
    transforms[i].transform.translation.y = 0;
    transforms[i].transform.translation.z = 0;

    Eigen::Quaternion<double> q;

    labust::tools::quaternionFromEulerZYX(
        rpy->data[3*i], 
        rpy->data[3*i+1],
        rpy->data[3*i+2],
        q);
    currentMeasQ[i] = q;
    if (isCalibrated) q = q * offsetQ[i];
    
    transforms[i].transform.rotation.w = q.w();
    transforms[i].transform.rotation.x = q.x();
    transforms[i].transform.rotation.y = q.y();
    transforms[i].transform.rotation.z = q.z();
    
    transforms[i].child_frame_id = "abs_" + names[i];
    transforms[i].header.frame_id = "local";
    transforms[i].header.stamp = ros::Time::now(); 
  }
	
  broadcast.sendTransform(transforms);

  for (int i=0; i<nodeCount; ++i) {
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
