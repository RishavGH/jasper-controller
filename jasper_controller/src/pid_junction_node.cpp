#include <fstream>
#include <iostream>
#include <string>

#include "jasper_controller/pid_junction_node.h"
#include "jasper_msgs/DynamicsInput.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "ros/package.h"
#include "ros/ros.h"

PID_Junction::PID_Junction(const ros::Publisher& pub_input, const ros::Publisher& pub_second)
  : pub(pub_input), err_pub(pub_second)
{
  Kp = Eigen::DiagonalMatrix<double, 6>();
  Kd = Eigen::DiagonalMatrix<double, 6>();

  // Reading PID params from file
  std::string file_path = ros::package::getPath("jasper_controller");
  file_path += "/resources/pid_params.txt";

  std::ifstream param_file(file_path);
  if (param_file.is_open())
  {
    // Kp
    std::string temp_str;

    for (int i = 0; i < 6; ++i)
    {
      param_file >> temp_str;
      Kp(i, i) = std::stod(temp_str);
    }

    // Kd
    for (int i = 0; i < 6; ++i)
    {
      param_file >> temp_str;
      Kd(i, i) = std::stod(temp_str);
    }
  }
  else
  {
    ROS_ERROR("Could not find pid_params.txt file! Please make sure that the file is placed in the resources folder of "
              "the jasper_controller package. Setting default PID params.");
    Kp.diagonal() << 100, 200, 500, 1200, 3300, 140000;
    Kd.diagonal() << 50, 50, 50, 70, 90, 80;
  }
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

  joint_err = stdVectorToMatrix(command_msg->jointAngles) - stdVectorToMatrix(feedback_msg->jointAngles);
  vel_err = stdVectorToMatrix(command_msg->jointVelocities) - stdVectorToMatrix(feedback_msg->jointVelocities);
  acc = stdVectorToMatrix(command_msg->jointAccel);

  DetermineCompensation(joint_err, vel_err, acc);

  jasper_msgs::DynamicsInput dyn_msg;
  jasper_msgs::JointInfo joint_err_msg;

  joint_err_msg.jointAngles = matrixToStdVector(joint_err);
  joint_err_msg.header.stamp = ros::Time::now();

  err_pub.publish(joint_err_msg);

  dyn_msg.jointAngles = feedback_msg->jointAngles;
  dyn_msg.jointVelocities = feedback_msg->jointVelocities;
  dyn_msg.accel = matrixToStdVector(jointPosCompensation + jointVelCompensation + jointAccCompensation);

  pub.publish(dyn_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pid_junction_node");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<jasper_msgs::DynamicsInput>("pid_command", 10);
  ros::Publisher error_pub =
      nh.advertise<jasper_msgs::JointInfo>("joint_error", 10);  // To tune the PID by plotting the error

  PID_Junction pid(pub, error_pub);

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