#include <iostream>

#include "jasper_controller/pid_junction_node.h"
#include "jasper_msgs/DynamicsInput.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "ros/ros.h"

PID_Junction::PID_Junction(const ros::Publisher& pub_input) : pub(pub_input)
{
  Kp = Eigen::DiagonalMatrix<double, 6>();
  Kd = Eigen::DiagonalMatrix<double, 6>();

  Kp.diagonal() << 100, 200, 200, 300, 300, 3000;
  Kd.diagonal() << 654, 780, 730, 1000, 1700, 13000;
}

template <typename Derived>
void PID_Junction::DetermineCompensation(const Eigen::MatrixBase<Derived>& joint_err,
                                         const Eigen::MatrixBase<Derived>& vel_err,
                                         const Eigen::MatrixBase<Derived>& acc)
{
  jointPosCompensation = Kp * joint_err;
  jointVelCompensation = Kd * vel_err;
  jointAccCompensation = acc;
}

void PID_Junction::pidCallback(const jasper_msgs::JointInfo::ConstPtr& command_msg,
                               const jasper_msgs::JointInfo::ConstPtr& feedback_msg)
{
  Eigen::Matrix<double, 6, 1> joint_err;
  Eigen::Matrix<double, 6, 1> vel_err;
  Eigen::Matrix<double, 6, 1> acc;

  std::cout << "PID Callback" << std::endl;

  joint_err = stdVectorToMatrix(command_msg->jointAngles) - stdVectorToMatrix(feedback_msg->jointAngles);
  vel_err = stdVectorToMatrix(command_msg->jointVelocities) - stdVectorToMatrix(feedback_msg->jointVelocities);
  acc = stdVectorToMatrix(command_msg->jointAccel);

  DetermineCompensation(joint_err, vel_err, acc);

  jasper_msgs::DynamicsInput dyn_msg;

  std::cout << jointVelCompensation << std::endl;

  dyn_msg.jointAngles = feedback_msg->jointAngles;
  dyn_msg.jointVelocities = feedback_msg->jointVelocities;
  dyn_msg.accel = matrixToStdVector(jointPosCompensation + jointVelCompensation + jointAccCompensation);

  pub.publish(dyn_msg);
}

Eigen::Matrix<double, 6, 1> PID_Junction::stdVectorToMatrix(const std::vector<double>& vec)
{
  const int cols = vec.size();
  Eigen::Matrix<double, 6, 1> result;

  for (int i = 0; i < vec.size(); ++i)
    result(i) = vec[i];

  return result;
}

std::vector<double> PID_Junction::matrixToStdVector(const Eigen::Matrix<double, 6, 1>& mat)
{
  std::vector<double> result;

  for (int i = 0; i < mat.rows(); ++i)
    result.push_back(mat(i));

  return result;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pid_junction_node");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<jasper_msgs::DynamicsInput>("pid_command", 10);

  PID_Junction pid(pub);

  std::cout << "PID Init" << std::endl;

  message_filters::Subscriber<jasper_msgs::JointInfo> joint_input_sub(nh, "joint_input", 10);
  message_filters::Subscriber<jasper_msgs::JointInfo> joint_feedback_sub(nh, "joint_feedback", 10);

  typedef message_filters::sync_policies::ApproximateTime<jasper_msgs::JointInfo, jasper_msgs::JointInfo>
      FeedbackSyncPolicy;

  message_filters::Synchronizer<FeedbackSyncPolicy> sync(FeedbackSyncPolicy(10), joint_input_sub, joint_feedback_sub);
  sync.registerCallback(boost::bind(&PID_Junction::pidCallback, &pid, _1, _2));

  ros::spin();

  return 0;
}