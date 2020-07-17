#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <vector>

struct JointLimits
{
  float lower_limit;
  float upper_limit;
};

class IK_Math
{
private:
  double a2, a3, d4, a6, dT;

  std::map<int, JointLimits> jointLimits;

public:
  // Constructors
  IK_Math();

  IK_Math(const std::vector<double>&);

  // Main IK function
  std::vector<double> IRB120_IK(const std::vector<double> position_input, const std::vector<double> lastConfig_input);

  // Functions to interchange between Eigen Vector and C++ STL vector
   Eigen::Matrix<double, 6, 1> stdVectorToMatrix(const std::vector<double>&);
   std::vector<double> matrixToStdVector(const Eigen::Matrix<double, 6, 1>&);

  // Function to interchange between euler angles to rotation matrix in the configuration specified by config (eg.
  // "XYZ", or "ZYZ" etc.)
  Eigen::Matrix3d eul2rotm(const Eigen::Vector3d&, std::string);

  // Function to check whether elements of a matrix is infinite
  template <typename Derived>
  Derived checkInf(const Eigen::MatrixBase<Derived>& mat);
};
