#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

#include "jasper_controller/robot.hpp"
#include "jasper_msgs/JointInfo.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

class AnalyticJacobian : private Robot
{
private:
  ros::Publisher forward_pub, inverse_pub;

  template <typename Derived>
  std::vector<double> calcAJacobian(const Eigen::MatrixBase<Derived>& jointAngles,
                                    const Eigen::MatrixBase<Derived>& jointVels);

  template <typename Derived>
  std::vector<double> calcInvAJacobian(const Eigen::MatrixBase<Derived>& jointAngles,
                                       const Eigen::MatrixBase<Derived>& action);

public:
  AnalyticJacobian(const ros::Publisher& fw_pub, const ros::Publisher& inv_pub, const ros::NodeHandle& nh);

  void ForwardCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void InverseCallback(const jasper_msgs::JointInfo::ConstPtr& msg);
};