#include "jasper_controller/robot.h"

std::vector<double> Robot::matrixToStdVector(const Eigen::Matrix<double, 6, 1>& mat)
{
  std::vector<double> result;

  for (int i = 0; i < mat.rows(); ++i)
    result.push_back(mat(i));

  return result;
}

Eigen::Matrix<double, 6, 1> Robot::stdVectorToMatrix(const std::vector<double>& vec)
{
  const int cols = vec.size();
  Eigen::Matrix<double, 6, 1> result;

  for (int i = 0; i < vec.size(); ++i)
    result(i) = vec[i];

  return result;
}

Robot::Robot()
{
}

Robot::Robot(const ros::NodeHandle& nh)
{
  nh.getParam("dh_a2", dh_a2_);
  nh.getParam("dh_a3", dh_a3_);
  nh.getParam("dh_d1", dh_d1_);
  nh.getParam("dh_d4", dh_d4_);
  nh.getParam("dh_a6", dh_a6_);
  nh.getParam("dh_dT", dh_dT_);
}