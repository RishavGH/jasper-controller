#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

#include "jasper_controller/robot.h"
#include "ros/ros.h"

class Starter : private Robot
{
private:
  Eigen::Matrix<double, 6, 7> waypoints;
  Eigen::Matrix<double, 6, 1> base2bowl;
  Eigen::Matrix<double, 6, 1> base2mouth;
  Eigen::Matrix<double, 6, 7> wpVels;
  Eigen::Matrix<double, 1, 7> timepoints;
  Eigen::Matrix<double, 6, 7> jointpoints;

  Eigen::Matrix<double, 6, 700> trajectory;
  Eigen::Matrix<double, 6, 700> jointVels;
  Eigen::Matrix<double, 6, 700> jointAccels;

  ros::Publisher pub;

public:
  Starter(const ros::Publisher& starter_pub);

  void cubicpolytraj();
  void InitJointPoints(ros::ServiceClient& client);
  void PublishJointPoints(int iteration_step);
};