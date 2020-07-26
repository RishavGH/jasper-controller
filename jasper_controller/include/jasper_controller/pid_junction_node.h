#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

#include "jasper_controller/robot.h"
#include "jasper_msgs/JointInfo.h"
#include "ros/ros.h"

class PID_Junction : private Robot
{
private:
  ros::Publisher pub;
  ros::Publisher err_pub;                            // To publish joint error
  Eigen::Matrix<double, 6, 1> jointPosCompensation;  // To store the outputs of the PID calculation
  Eigen::Matrix<double, 6, 1> jointVelCompensation;
  Eigen::Matrix<double, 6, 1> jointAccCompensation;

  Eigen::Matrix<double, 6, 6> Kp;
  Eigen::Matrix<double, 6, 6> Kd;

  template <typename Derived>
  void DetermineCompensation(const Eigen::MatrixBase<Derived>& joint_err, const Eigen::MatrixBase<Derived>& vel_err,
                             const Eigen::MatrixBase<Derived>& acc);
  /*Eigen::Matrix<double, 6, 1> stdVectorToMatrix(const std::vector<double>& vec);
  std::vector<double> matrixToStdVector(const Eigen::Matrix<double, 6, 1>& mat);*/

public:
  PID_Junction(const ros::Publisher& pub_input, const ros::Publisher& pub_second);
  void pidCallback(const jasper_msgs::JointInfo::ConstPtr& command_msg,
                   const jasper_msgs::JointInfo::ConstPtr& feedback_msg);
};