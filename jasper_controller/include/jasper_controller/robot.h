/****************************************************************************
 * Robot
 * A colection of functions that are commonly used in the jasper_controller.
 ****************************************************************************/

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

#include "ros/ros.h"

class Robot
{
protected:
  double dh_a2_, dh_a3_, dh_d4_, dh_d1_, dh_a6_, dh_dT_;  // DH Parameters

  std::vector<double> matrixToStdVector(const Eigen::Matrix<double, 6, 1>& mat);
  Eigen::Matrix<double, 6, 1> stdVectorToMatrix(const std::vector<double>& vec);

  Robot(const ros::NodeHandle& nh);
  Robot();
};
