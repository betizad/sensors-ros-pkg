#include <labust/sensors/KinectTransformPublisherNode.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

using namespace labust::sensors;

#define CHECK_RC(nRetVal, what)                                   \
  if (nRetVal != XN_STATUS_OK) {                                  \
    ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal)); \
    return nRetVal;                                               \
  }


KinectTransformPublisherNode::KinectTransformPublisherNode() :
    g_bNeedPose(false),
    tf_listener(tf_buffer) {
}

KinectTransformPublisherNode::~KinectTransformPublisherNode() {}

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
  ROS_INFO("New User %d", nId);

  KinectTransformPublisherNode *ktp = static_cast<KinectTransformPublisherNode*>(pCookie);
  if (ktp->g_bNeedPose)
    ktp->g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(ktp->g_strPose, nId);
  else
    ktp->g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

namespace {
  void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
    ROS_INFO("Lost user %d", nId);
  }

  void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
    ROS_INFO("Calibration started for user %d", nId);
  }

  void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
    KinectTransformPublisherNode *ktp = static_cast<KinectTransformPublisherNode*>(pCookie);
    if (bSuccess) {
      ROS_INFO("Calibration complete, start tracking user %d", nId);
      ktp->g_UserGenerator.GetSkeletonCap().StartTracking(nId);
    }
    else {
      ROS_INFO("Calibration failed for user %d", nId);
      if (ktp->g_bNeedPose)
        ktp->g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(ktp->g_strPose, nId);
      else
        ktp->g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
    }
  }
}

int KinectTransformPublisherNode::start() {
  ros::NodeHandle nh, ph("~");
  ros::Rate rate(1);
  
  std::string configFilename = ros::package::getPath("labust_kinect") + "/openni_settings.xml";
  xn::ScriptNode sNode;
  XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str(), sNode);
  CHECK_RC(nRetVal, "InitFromXml");

  nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
  if (nRetVal != XN_STATUS_OK) {
    nRetVal = g_UserGenerator.Create(g_Context);
    if (nRetVal != XN_STATUS_OK) {
      ROS_ERROR("NITE is likely missing: Please install NITE >= 1.5.2.21. " 
          "Check the readme for download information. "
          "Error Info: User generator failed: %s", xnGetStatusString(nRetVal));
      return nRetVal;
    }
  }

  if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
    ROS_INFO("Supplied user generator doesn't support skeleton");
    return 1;
  }

  XnCallbackHandle hUserCallbacks;
  g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, this, hUserCallbacks);

  XnCallbackHandle hCalibrationCallbacks;
  g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(
      UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, this, hCalibrationCallbacks);

  

}


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "kinect_transform_publisher_node");
  KinectTransformPublisherNode node;
  return node.start();
}
