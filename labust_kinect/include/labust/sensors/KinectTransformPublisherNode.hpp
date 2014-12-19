#ifndef KINECTTRANSFORMPUBLISHERNODE_HPP_
#define KINECTTRANSFORMPUBLISHERNODE_HPP_
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <map>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

namespace labust 
{
  namespace sensors 
  {
    /**
     */
    class KinectTransformPublisherNode {

      public:
        /**
         * Main constructor
         */
        KinectTransformPublisherNode();
        /**
         * Generic destructor.
         */
        ~KinectTransformPublisherNode();
        int start();
        xn::Context        g_Context;
        xn::DepthGenerator g_DepthGenerator;
        xn::UserGenerator  g_UserGenerator;

        XnBool g_bNeedPose;
        XnChar g_strPose[20];

      private:
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;
    
    };
  }
}

/* KINECTTRANSFORMPUBLISHERNODE_HPP_ */
#endif
