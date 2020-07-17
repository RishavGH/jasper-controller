#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

class Feedback
{
private:
  ros::Publisher pub;

public:
  Feedback(const ros::Publisher& input_pub);
  void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
};