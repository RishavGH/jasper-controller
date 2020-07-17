#include <ros/ros.h>

#include <cmath>
#include <iostream>
#include <limits>

#include "inverse_kinematics/ik_math.h"

IK_Math::IK_Math()
{
  // DH parameters
  a2 = 0.27;
  a3 = 0.07;
  d4 = 0.302;
  a6 = 0.046;
  dT = 0.2069;

  // Defining joint limits
  jointLimits = { { 1, { -2.87979, 2.87979 } }, { 2, { -1.91986, 1.91986 } },   { 3, { -1.91986, 1.22173 } },
                  { 4, { -2.79253, 2.79253 } }, { 5, { -2.094395, 2.094395 } }, { 6, { -6.98132, 6.98132 } } };
}

IK_Math::IK_Math(const std::vector<double> &dh_params_)
{
  // DH parameters
  a2 = dh_params_[0];
  a3 = dh_params_[1];
  d4 = dh_params_[2];
  a6 = dh_params_[3];
  dT = dh_params_[4];

  // Defining joint limits
  jointLimits = { { 1, { -2.87979, 2.87979 } }, { 2, { -1.91986, 1.91986 } },   { 3, { -1.91986, 1.22173 } },
                  { 4, { -2.79253, 2.79253 } }, { 5, { -2.094395, 2.094395 } }, { 6, { -6.98132, 6.98132 } } };
}

std::vector<double> IK_Math::IRB120_IK(const std::vector<double> position_input,
                                       const std::vector<double> lastConfig_input)
{
  // Converting input vectors to Eigen vectors for easier processing

  Eigen::Matrix<double, 6, 1> position;
  position = stdVectorToMatrix(position_input);

  std::cout << position << std::endl;

  Eigen::Matrix<double, 6, 1> lastConfig;
  lastConfig = stdVectorToMatrix(lastConfig_input);

  // The resultant vector
  Eigen::Matrix<double, 1, 6> poseVector;

  // Breaking the input position into linear coordinates and rotational coordinates
  Eigen::Vector3d lin_pos = position.block<3, 1>(0, 0);     // linear
  Eigen::Vector3d eul_angles = position.block<3, 1>(3, 0);  // rotational

  // Converting Euler angles to rotation matrix

  Eigen::Matrix3d rotm;
  rotm = eul2rotm(eul_angles, "XYZ");

  // Additional vectors for further processing
  Eigen::Vector3d a_end = rotm.block<3, 1>(0, 2);
  Eigen::Vector3d n_end = rotm.block<3, 1>(0, 0);

  // Extracting cartesian coordinates and modifying
  double x = lin_pos(0) - dT * a_end(0) - a6 * n_end(0);
  double y = lin_pos(1) - dT * a_end(1) - a6 * n_end(1);
  double z = lin_pos(2) - dT * a_end(2) - a6 * n_end(2) - 0.29;

  // Helper values
  double r = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
  double r1 = sqrt(pow(x, 2) + pow(y, 2));
  double r2 = sqrt(pow(d4, 2) + pow(a3, 2));

  //======================== JOINTS 1 TO 3 ======================================================
  // Joint 1
  double theta1 = atan2(y, x);

  if (theta1 < jointLimits[1].lower_limit || theta1 > jointLimits[1].upper_limit)
    theta1 = std::numeric_limits<double>::infinity();

  // Joint 3
  double theta3a = 0.0, theta3b = 0.0;

  theta3a =
      atan2(sqrt(pow(a3, 2) + pow(d4, 2) - pow(((pow(r, 2) - pow(a2, 2) - pow(a3, 2) - pow(d4, 2)) / (2 * a2)), 2)),
            (pow(r, 2) - pow(a2, 2) - pow(a3, 2) - pow(d4, 2)) / (2 * a2)) -
      atan2(d4, a3);
  theta3b =
      atan2(-sqrt(pow(a3, 2) + pow(d4, 2) - pow(((pow(r, 2) - pow(a2, 2) - pow(a3, 2) - pow(d4, 2)) / (2 * a2)), 2)),
            (pow(r, 2) - pow(a2, 2) - pow(a3, 2) - pow(d4, 2)) / (2 * a2)) -
      atan2(d4, a3);

  if ((theta3a - jointLimits[3].lower_limit) * (jointLimits[3].upper_limit - theta3a) < 0)
    theta3a = std::numeric_limits<double>::infinity();
  if ((theta3a - jointLimits[3].lower_limit) * (jointLimits[3].upper_limit - theta3a) < 0)
    theta3b = std::numeric_limits<double>::infinity();

  // Joint 2
  double theta2a = 0.0, theta2b = 0.0;

  double alpha = atan2(z / r, r1 / r);
  double betaa = atan2(sqrt(1 - pow(((pow(a2, 2) + pow(r, 2) - pow(r2, 2)) / (2 * a2 * r)), 2)),
                       (pow(a2, 2) + pow(r, 2) - pow(r2, 2)) / (2 * a2 * r));
  double betab = atan2(-sqrt(1 - pow(((pow(a2, 2) + pow(r, 2) - pow(r2, 2)) / (2 * a2 * r)), 2)),
                       (pow(a2, 2) + pow(r, 2) - pow(r2, 2)) / (2 * a2 * r));

  theta2a = M_PI / 2 - alpha - betaa;
  theta2b = M_PI / 2 - alpha - betab;

  if ((theta2a - jointLimits[2].lower_limit) * (jointLimits[2].upper_limit - theta2a) < 0)
    theta2a = std::numeric_limits<double>::infinity();
  if ((theta2b - jointLimits[2].lower_limit) * (jointLimits[2].upper_limit - theta2b) < 0)
    theta2b = std::numeric_limits<double>::infinity();

  // Compiling solutions into one matrix for further processing
  Eigen::Matrix<double, 4, 3> soln_mat_linear;
  soln_mat_linear << theta1, theta2a, theta3a, theta1, theta2a, theta3b, theta1, theta2b, theta3a, theta1, theta2b,
      theta3b;

  // Choosing the pose closest to last pose and ignoring solutions with infinity

  int minrow = 0;
  double minval = std::numeric_limits<double>::infinity();

  for (int row = 0; row < 4; ++row)
  {
    Eigen::Matrix<double, 1, 3> soln_vec = soln_mat_linear.block<1, 3>(row, 0);
    Eigen::Matrix<double, 1, 3> check_vec = checkInf(soln_vec);

    if (!check_vec.sum())
    {
      double val =
          sqrt(pow(soln_mat_linear(row, 0) - lastConfig(0), 2) + pow(soln_mat_linear(row, 1) - lastConfig(1), 2) +
               pow(soln_mat_linear(row, 2) - lastConfig(2), 2));

      if (val < minval)
      {
        minval = val;
        minrow = row;
      }
    }
  }

  // Assigning the cartesian coordinates to the resultant vector
  poseVector.block<1, 3>(0, 0) = soln_mat_linear.block<1, 3>(minrow, 0);

  theta1 = soln_mat_linear(minrow, 0);
  double theta2 = soln_mat_linear(minrow, 1);
  double theta3 = soln_mat_linear(minrow, 2);

  //=============================================================================================
  //======================== JOINTS 4 TO 6 ======================================================

  // Joint 5
  double s5sq =
      pow((rotm(2, 2) * cos(theta2) * cos(theta3) - rotm(2, 2) * sin(theta2) * sin(theta3) +
           rotm(0, 2) * cos(theta1) * cos(theta2) * sin(theta3) + rotm(0, 2) * cos(theta1) * cos(theta3) * sin(theta2) +
           rotm(1, 2) * cos(theta2) * sin(theta1) * sin(theta3) + rotm(1, 2) * cos(theta3) * sin(theta1) * sin(theta2)),
          2) +
      pow((rotm(1, 2) * cos(theta1) - rotm(0, 2) * sin(theta1)), 2);
  double at5a = sqrt(s5sq);
  double at5b = rotm(0, 2) * cos(theta1) * cos(theta2) * cos(theta3) - rotm(2, 2) * cos(theta3) * sin(theta2) -
                rotm(2, 2) * cos(theta2) * sin(theta3) + rotm(1, 2) * cos(theta2) * cos(theta3) * sin(theta1) -
                rotm(0, 2) * cos(theta1) * sin(theta2) * sin(theta3) -
                rotm(1, 2) * sin(theta1) * sin(theta2) * sin(theta3);

  double theta5a = atan2(at5a, at5b);
  double theta5b = atan2(-at5a, at5b);

  if ((theta5a - jointLimits[5].lower_limit) * (jointLimits[5].upper_limit - theta5a) < 0)
    theta5a = std::numeric_limits<double>::infinity();
  if ((theta5b - jointLimits[5].lower_limit) * (jointLimits[5].upper_limit - theta5b) < 0)
    theta5b = std::numeric_limits<double>::infinity();

  // Joint 4

  double theta4a = 0, theta4b = 0;

  double at4b =
      -(rotm(2, 2) * cos(theta2) * cos(theta3) - rotm(2, 2) * sin(theta2) * sin(theta3) +
        rotm(0, 2) * cos(theta1) * cos(theta2) * sin(theta3) + rotm(0, 2) * cos(theta1) * cos(theta3) * sin(theta2) +
        rotm(1, 2) * cos(theta2) * sin(theta1) * sin(theta3) + rotm(1, 2) * cos(theta3) * sin(theta1) * sin(theta2));
  double at4a = rotm(1, 2) * cos(theta1) - rotm(0, 2) * sin(theta1);

  if (abs(at4a) < 1e-11 && abs(at4b) < 1e-11)
  {
    theta4a = 0;
    theta4b = 0;
  }
  else
  {
    theta4a = atan2(at4a / sin(theta5a), at4b / sin(theta5a));
    theta4b = atan2(at4a / sin(theta5b), at4b / sin(theta5b));
  }

  if ((theta4a - jointLimits[4].lower_limit) * (jointLimits[4].upper_limit - theta4a) < 0)
    theta4a = std::numeric_limits<double>::infinity();
  if ((theta4b - jointLimits[4].lower_limit) * (jointLimits[4].upper_limit - theta4b) < 0)
    theta4b = std::numeric_limits<double>::infinity();

  // Joint 6

  double theta6a = 0, theta6b = 0;

  double at6a =
      (rotm(0, 1) * cos(theta1) * cos(theta2) * cos(theta3) - rotm(2, 1) * cos(theta3) * sin(theta2) -
       rotm(2, 1) * cos(theta2) * sin(theta3) + rotm(1, 1) * cos(theta2) * cos(theta3) * sin(theta1) -
       rotm(0, 1) * cos(theta1) * sin(theta2) * sin(theta3) - rotm(1, 1) * sin(theta1) * sin(theta2) * sin(theta3));
  double at6b =
      -(rotm(0, 0) * cos(theta1) * cos(theta2) * cos(theta3) - rotm(2, 0) * cos(theta3) * sin(theta2) -
        rotm(2, 0) * cos(theta2) * sin(theta3) + rotm(1, 0) * cos(theta2) * cos(theta3) * sin(theta1) -
        rotm(0, 0) * cos(theta1) * sin(theta2) * sin(theta3) - rotm(1, 0) * sin(theta1) * sin(theta2) * sin(theta3));

  if (abs(at6a) < 1e-11 && abs(at6b) < 1e-11)
  {
    theta6a = 0;
    theta6b = 0;
  }
  else
  {
    theta6a = atan2(at6a / sin(theta5a), at6b / sin(theta5a));
    theta6b = atan2(at6a / sin(theta5b), at6b / sin(theta5b));
  }

  if ((theta6a - jointLimits[6].lower_limit) * (jointLimits[6].upper_limit - theta6a) < 0)
    theta6a = std::numeric_limits<double>::infinity();
  if ((theta6b - jointLimits[6].lower_limit) * (jointLimits[6].upper_limit - theta6b) < 0)
    theta6b = std::numeric_limits<double>::infinity();

  Eigen::Matrix<double, 2, 3> soln_mat_or;
  soln_mat_or << theta4a, theta5a, theta6a, theta4b, theta5b, theta6b;

  // Choosing pose closest to last pose
  minrow = 0;
  minval = std::numeric_limits<double>::infinity();

  for (int row = 0; row < 2; ++row)
  {
    Eigen::Matrix<double, 1, 3> soln_vec = soln_mat_or.block<1, 3>(row, 0);

    Eigen::Matrix<double, 1, 3> check_vec = checkInf(soln_vec);

    if (!check_vec.sum())
    {
      double val = sqrt(pow(soln_mat_or(row, 0) - lastConfig(3), 2) + pow(soln_mat_or(row, 1) - lastConfig(4), 2) +
                        pow(soln_mat_or(row, 2) - lastConfig(5), 2));

      if (val < minval)
      {
        minval = val;
        minrow = row;
      }
    }
  }

  poseVector.block<1, 3>(0, 3) = soln_mat_or.block<1, 3>(minrow, 0);

  //=============================================================================================

  std::vector<double> result;
  result = matrixToStdVector(poseVector);

  return result;
}

template <typename Derived>
Derived IK_Math::checkInf(const Eigen::MatrixBase<Derived> &mat)
{
  const int rows = mat.rows();
  const int cols = mat.cols();

  Derived result;

  for (int row = 0; row < rows; ++row)
    for (int col = 0; col < cols; ++col)
    {
      if (isinf(mat(row, col)))
        result(row, col) = 1;
      else
        result(row, col) = 0;
    }

  return result;
}

// Convert C++ STL vector to Eigen Vector (1D Matrix)
Eigen::Matrix<double, 6, 1> IK_Math::stdVectorToMatrix(const std::vector<double> &vec)
{
  const int cols = vec.size();
  Eigen::Matrix<double, 6, 1> result;

  for (int i = 0; i < vec.size(); ++i)
    result(i) = vec[i];

  return result;
}

// Convert from Eigen Vector (1D Matrix) to C++ STL vector
std::vector<double> IK_Math::matrixToStdVector(const Eigen::Matrix<double, 6, 1> &mat)
{
  std::vector<double> result;

  for (int i = 0; i < mat.rows(); ++i)
    result.push_back(mat(i));

  return result;
}

// Convert euler angles to rotation matrix in "config" configuration (i.e. "XYZ" or "ZYZ" etc.)
Eigen::Matrix3d IK_Math::eul2rotm(const Eigen::Vector3d &eul_angles, std::string config)
{
  Eigen::Matrix3d rotm_result;

  // Declaring placeholders for unit vectors to be used in the euler sequence
  Eigen::Vector3d first;
  Eigen::Vector3d second;
  Eigen::Vector3d third;

  // Assigning unit vectors according to the config sequence specified

  switch (config[0])
  {
    case 'X':
      first = Eigen::Vector3d::UnitX();
      break;
    case 'x':
      first = Eigen::Vector3d::UnitX();
      break;
    case 'Y':
      first = Eigen::Vector3d::UnitY();
      break;
    case 'y':
      first = Eigen::Vector3d::UnitY();
      break;
    case 'Z':
      first = Eigen::Vector3d::UnitZ();
      break;
    case 'z':
      first = Eigen::Vector3d::UnitZ();
      break;
    default:
      std::cout << "Axis specified in first character of configuration sequence is not recognized (Must be X,Y or Z)"
                << std::endl;
  }

  switch (config[1])
  {
    case 'X':
      second = Eigen::Vector3d::UnitX();
      break;
    case 'x':
      second = Eigen::Vector3d::UnitX();
      break;
    case 'Y':
      second = Eigen::Vector3d::UnitY();
      break;
    case 'y':
      second = Eigen::Vector3d::UnitY();
      break;
    case 'Z':
      second = Eigen::Vector3d::UnitZ();
      break;
    case 'z':
      second = Eigen::Vector3d::UnitZ();
      break;
    default:
      std::cout << "Axis specified in second character of configuration sequence is not recognized (Must be X,Y or Z)"
                << std::endl;
  }

  switch (config[2])
  {
    case 'X':
      third = Eigen::Vector3d::UnitX();
      break;
    case 'x':
      third = Eigen::Vector3d::UnitX();
      break;
    case 'Y':
      third = Eigen::Vector3d::UnitY();
      break;
    case 'y':
      third = Eigen::Vector3d::UnitY();
      break;
    case 'Z':
      third = Eigen::Vector3d::UnitZ();
      break;
    case 'z':
      third = Eigen::Vector3d::UnitZ();
      break;
    default:
      std::cout << "Axis specified in third character of configuration sequence is not recognized (Must be X,Y or Z)"
                << std::endl;
  }

  // Using AngleAxis to create the final rotation matrix
  rotm_result = Eigen::AngleAxisd(eul_angles(0), first) * Eigen::AngleAxisd(eul_angles(1), second) *
                Eigen::AngleAxisd(eul_angles(2), third);

  return rotm_result;
}
