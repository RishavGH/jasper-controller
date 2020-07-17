#include <iostream>

#include "jasper_controller/torque_middleman_node.h"

TorqueMiddleman::TorqueMiddleman(const ros::Publisher* pub_ptr) : pub_array(pub_ptr)
{
}

void TorqueMiddleman::CommandCallback(const jasper_msgs::DynamicsOutput::ConstPtr& msg) const
{
  std::vector<double> torques = msg->jointTorques;

  std_msgs::Float64 oneTorque;

  for (int i = 0; i < torques.size(); ++i)
  {
    oneTorque.data = torques[i];
    std::cout << oneTorque.data << " ";
    pub_array[i].publish(oneTorque);
  }

  std::cout << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "torque_middleman_node");

  ros::NodeHandle nh;

  ros::Publisher* pub_array = new ros::Publisher[6];

  pub_array[0] = nh.advertise<std_msgs::Float64>("joint1_controller/command", 10, true);
  pub_array[1] = nh.advertise<std_msgs::Float64>("joint2_controller/command", 10, true);
  pub_array[2] = nh.advertise<std_msgs::Float64>("joint3_controller/command", 10, true);
  pub_array[3] = nh.advertise<std_msgs::Float64>("joint4_controller/command", 10, true);
  pub_array[4] = nh.advertise<std_msgs::Float64>("joint5_controller/command", 10, true);
  pub_array[5] = nh.advertise<std_msgs::Float64>("joint6_controller/command", 10, true);

  TorqueMiddleman tm(pub_array);

  ros::Subscriber sub = nh.subscribe("torque_command", 10, &TorqueMiddleman::CommandCallback, &tm);

  ros::spin();

  delete[] pub_array;

  return 0;
}