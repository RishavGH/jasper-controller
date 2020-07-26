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