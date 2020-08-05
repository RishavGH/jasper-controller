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

  template <typename Derived1, typename Derived2, typename Derived3, typename Derived4>
  void cubicpolytraj(const Eigen ::MatrixBase<Derived1>& waypoints, const Eigen ::MatrixBase<Derived1>& wpVels,
                     const Eigen ::MatrixBase<Derived2>& timepoints, const Eigen::MatrixBase<Derived3>& tSamples,
                     Eigen::MatrixBase<Derived4>& q, Eigen::MatrixBase<Derived4>& qd, Eigen::MatrixBase<Derived4>& qdd);

  Robot(const ros::NodeHandle& nh);
  Robot();
};

std::vector<double> Robot::matrixToStdVector(const Eigen::Matrix<double, 6, 1>& mat)
{
  std::vector<double> result;
  result.reserve(6);

  for (int i = 0; i < mat.rows(); ++i)
    result.emplace_back(mat(i));

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

template <typename Derived1, typename Derived2, typename Derived3, typename Derived4>
void Robot::cubicpolytraj(const Eigen ::MatrixBase<Derived1>& waypoints, const Eigen ::MatrixBase<Derived1>& wpVels,
                          const Eigen ::MatrixBase<Derived2>& timepoints, const Eigen::MatrixBase<Derived3>& tSamples,
                          Eigen::MatrixBase<Derived4>& q, Eigen::MatrixBase<Derived4>& qd,
                          Eigen::MatrixBase<Derived4>& qdd)
{
  // To solve for polynomial coefficients using the matrix method
  Eigen::Matrix4d variable_power_matrix;
  Eigen::Matrix<double, 4, 6> right_hand_side;
  Eigen::Matrix<double, 4, 6> coeff_matrix;
  Eigen::Matrix<double, 3, 4> result_matrix;  // Matrix of time powers for calculating the final results
  Eigen::Matrix<double, 3, 6> traj_matrix;

  for (int i = 0, j = 0; i < timepoints.cols() - 1; ++i)
  {
    // For calculating the coefficients
    variable_power_matrix << pow(timepoints(i), 3), pow(timepoints(i), 2), pow(timepoints(i), 1), 1,
        pow(timepoints(i + 1), 3), pow(timepoints(i + 1), 2), pow(timepoints(i + 1), 1), 1, 3 * pow(timepoints(i), 2),
        2 * pow(timepoints(i), 1), 1, 0, 3 * pow(timepoints(i + 1), 2), 2 * pow(timepoints(i + 1), 1), 1, 0;

    right_hand_side.row(0) = waypoints.col(i).transpose();
    right_hand_side.row(1) = waypoints.col(i + 1).transpose();
    right_hand_side.row(2) = wpVels.col(i).transpose();
    right_hand_side.row(3) = wpVels.col(i + 1).transpose();

    coeff_matrix = variable_power_matrix.inverse() * right_hand_side;

    // Calculating the trajectory, vel and acceleration

    for (; tSamples(j) < timepoints(i + 1); ++j)
    {
      result_matrix << pow(tSamples(j), 3), pow(tSamples(j), 2), pow(tSamples(j), 1), 1, 3 * pow(tSamples(j), 2),
          2 * pow(tSamples(j), 1), 1, 0, 6 * tSamples(j), 2, 0, 0;

      traj_matrix = result_matrix * coeff_matrix;

      q.col(j) = traj_matrix.row(0).transpose();
      qd.col(j) = traj_matrix.row(1).transpose();
      qdd.col(j) = traj_matrix.row(2).transpose();
    }
  }
}
