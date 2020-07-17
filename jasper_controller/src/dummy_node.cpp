#include <vector>

#include "jasper_msgs/JointInfo.h"
#include "ros/ros.h"

class Dummy
{
private:
  ros::Publisher pub;

public:
  Dummy(const ros::Publisher& input_pub) : pub(input_pub)
  {
  }

  void DummyCallback(const jasper_msgs::JointInfo::ConstPtr& msg)
  {
    std::vector<double> jointPos = msg->jointAngles;
    std::vector<double> jointVel = msg->jointVelocities;

    jasper_msgs::JointInfo outgoing_msg;

    for (int i = 0; i < 6; ++i)
    {
      jointPos[i] = jointPos[i] + 1;
      jointVel[i] = jointVel[i] + 1;
    }

    outgoing_msg.jointAngles = jointPos;
    outgoing_msg.jointVelocities = jointVel;
    std::cout << "Hello" << std::endl;
    outgoing_msg.header.stamp = msg->header.stamp;
    pub.publish(outgoing_msg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dummy_node");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<jasper_msgs::JointInfo>("joint_feedback", 10);
  Dummy dummy(pub);

  ros::Subscriber sub = nh.subscribe("joint_input", 10, &Dummy::DummyCallback, &dummy);

  ros::spin();

  return 0;
}