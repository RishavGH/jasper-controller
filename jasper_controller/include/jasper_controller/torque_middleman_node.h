#include <vector>

#include "jasper_msgs/DynamicsOutput.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"

class TorqueMiddleman
{
private:
  const ros::Publisher* pub_array;

public:
  TorqueMiddleman(const ros::Publisher* pub_ptr);

  void CommandCallback(const jasper_msgs::DynamicsOutput::ConstPtr& msg) const;
};