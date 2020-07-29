#include "jasper_msgs/JointInfo.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

class Feedback
{
private:
  ros::Publisher pub;

public:
  Feedback(const ros::Publisher& input_pub);
  void JointStateCallback(const jasper_msgs::JointInfo::ConstPtr& msg);
};