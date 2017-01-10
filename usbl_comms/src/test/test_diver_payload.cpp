#include <caddy_msgs/DiverPayload.h>
#include <ros/ros.h>

#include <boost/bind.hpp>

class DiverPayloadPub
{
public:
  DiverPayloadPub()
  {
    this->onInit();
  };

  void onInit()
  {
    ros::NodeHandle nh;

    pay_out = nh.advertise<caddy_msgs::DiverPayload>("diver_info", 1);
  }

  void start()
  {
    ros::Rate rate(5);
    while (ros::ok())
    {
      caddy_msgs::DiverPayload payload;
      // TODO: Add randomized values
      payload.alarm = 1;
      payload.average_flipper_rate = 2.2;
      payload.hearth_rate = 122;
      payload.breathing_rate = 21;
      payload.motion_rate = 2;
      payload.pad_space = 20;

      pay_out.publish(payload);
      rate.sleep();
      ros::spinOnce();
    }
  }

private:
  /// The publisher of the payload topic.
  ros::Publisher pay_out;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_diver_payload");

  DiverPayloadPub payload;
  payload.start();
}
