#include <iostream>
#include <vector>

#include "jasper_controller/feedback_node.h"
#include "jasper_msgs/JointInfo.h"

Feedback::Feedback(const ros::Publisher& input_pub) : pub(input_pub)
{
}

void Feedback::JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  std::vector<double> jointStates = msg->position;
  std::vector<double> jointVel = msg->velocity;

  jasper_msgs::JointInfo outgoing_msg;

  outgoing_msg.jointAngles = jointStates;
  outgoing_msg.jointVelocities = jointVel;

  for (int i = 0; i < 6; ++i)
  {
    std::cout << jointStates[i] << " ";
  }
  std::cout << std::endl;

  outgoing_msg.header.stamp = ros::Time::now();

  pub.publish(outgoing_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "feedback_node");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<jasper_msgs::JointInfo>("joint_feedback", 10);

  Feedback feedback(pub);

  ros::Subscriber sub = nh.subscribe("joint_states", 10, &Feedback::JointStateCallback, &feedback);

  ros::spin();

  return 0;
}