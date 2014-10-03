#ifndef DIVERNETBVHPUBLISHERNODE_HPP_
#define DIVERNETBVHPUBLISHERNODE_HPP_
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <map>

namespace labust 
{
  namespace sensors 
  {
    /**
     * Class for publishing joint state data as a BVH model.
     * Subscribes to joint_states topic and prints BVH data on stdout.
     */
    class DiverNetBvhPublisherNode {

      public:
        /**
         * Main constructor
         */
        DiverNetBvhPublisherNode();
        /**
         * Generic destructor.
         */
        ~DiverNetBvhPublisherNode();

      private:
        void onInit();
        void processData(const sensor_msgs::JointState &joint_states);
        ros::Subscriber joint_states_subscriber;
        std::vector<std::vector<float> > data;
        std::map<std::string, int> joint_map;
        std::string bvh_model_file;
        double framerate;
        int node_count, num_frames;
    };
  }
}

/* DIVERNETBVHPUBLISHERNODE_HPP_ */
#endif
