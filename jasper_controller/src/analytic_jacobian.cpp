#include "jasper_controller/analytic_jacobian.h"
#include "jasper_msgs/DynamicsInput.h"

AnalyticJacobian::AnalyticJacobian(const ros::Publisher& fw_pub, const ros::Publisher& inv_pub,
                                   const ros::NodeHandle& nh)
  : Robot(nh), forward_pub(fw_pub), inverse_pub(inv_pub)
{
}

template <typename Derived>
std::vector<double> AnalyticJacobian::calcAJacobian(const Eigen::MatrixBase<Derived>& jointAngles,
                                                    const Eigen::MatrixBase<Derived>& jointVels)
{
  return matrixToStdVector(jointAngles);
}

template <typename Derived>
std::vector<double> AnalyticJacobian::calcInvAJacobian(const Eigen::MatrixBase<Derived>& jointAngles,
                                                       const Eigen::MatrixBase<Derived>& action)
{
  return matrixToStdVector(jointAngles);
}

void AnalyticJacobian::ForwardCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  std::vector<double> feedPos = msg->position;
  std::vector<double> feedVels = msg->velocity;

  Eigen::Matrix<double, 6, 1> jointAngles = stdVectorToMatrix(feedPos);
  Eigen::Matrix<double, 6, 1> jointVels = stdVectorToMatrix(feedVels);

  std::vector<double> result = calcAJacobian(jointAngles, jointVels);

  jasper_msgs::JointInfo back_msg;
  // back_msg.jointAngles = feedPos;
  back_msg.jointAngles = { 0, 0, 0, 0, 0, 0 };
  back_msg.jointVelocities = result;

  back_msg.header.stamp = ros::Time::now();
  forward_pub.publish(back_msg);
}

void AnalyticJacobian::InverseCallback(const jasper_msgs::JointInfo::ConstPtr& msg)
{
  std::vector<double> feedPos = msg->jointAngles;
  std::vector<double> feedVels = msg->jointVelocities;
  std::vector<double> feedAcc = msg->jointAccel;

  jasper_msgs::DynamicsInput dyn_msg;
  dyn_msg.jointAngles = feedPos;
  dyn_msg.jointVelocities = feedVels;

  Eigen::Matrix<double, 6, 1> pos = stdVectorToMatrix(feedPos);
  Eigen::Matrix<double, 6, 1> action = stdVectorToMatrix(feedAcc);

  dyn_msg.accel = calcInvAJacobian(pos, action);

  inverse_pub.publish(dyn_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "analytic_jacobian_node");

  ros::NodeHandle nh;

  ros::Publisher forward_publisher = nh.advertise<jasper_msgs::JointInfo>("op_feedback", 10);
  ros::Publisher inverse_publisher = nh.advertise<jasper_msgs::DynamicsInput>("resolved_command", 10);

  AnalyticJacobian ajac(forward_publisher, inverse_publisher, nh);

  ros::Subscriber feedback_sub = nh.subscribe("joint_states", 10, &AnalyticJacobian::ForwardCallback, &ajac);
  ros::Subscriber forward_sub = nh.subscribe("pid_command", 10, &AnalyticJacobian::InverseCallback, &ajac);

  ros::spin();

  return 0;
}