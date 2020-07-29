#include <iostream>
#include <vector>

#include "jasper_controller/feedback_node.h"
#include "jasper_msgs/FKService.h"
#include "jasper_msgs/JointInfo.h"

Feedback::Feedback(const ros::Publisher& input_pub, const ros::ServiceClient& input_srv)
  : pub(input_pub), srv(input_srv)
{
}

void Feedback::JointStateCallback(const jasper_msgs::JointInfo::ConstPtr& msg)
{
  jasper_msgs::FKServiceRequest srv_req;
  jasper_msgs::FKServiceResponse srv_res;

  std::vector<double> jointStates = msg->jointAngles;
  std::vector<double> jointVel = msg->jointVelocities;

  srv_req.jointAngles = jointStates;
  std::vector<double> eePos;

  if (srv.call(srv_req, srv_res))
  {
    eePos = srv_res.eePos;
  }
  else
  {
    ROS_ERROR("Failed to call service forward_kinematics_service!");
  }

  jasper_msgs::JointInfo outgoing_msg;

  outgoing_msg.jointAngles = eePos;
  outgoing_msg.jointVelocities = jointVel;

  for (int i = 0; i < 6; ++i)
  {
    std::cout << eePos[i] << " ";
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
  ros::ServiceClient srv = nh.serviceClient<jasper_msgs::FKService>("forward_kinematics_service");

  Feedback feedback(pub, srv);

  ros::Subscriber sub = nh.subscribe("op_feedback", 10, &Feedback::JointStateCallback, &feedback);

  ros::spin();

  return 0;
}