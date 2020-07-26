#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <map>
#include <vector>

#include "jasper_controller/robot.h"
#include "jasper_msgs/DynamicsInput.h"
#include "jasper_msgs/DynamicsOutput.h"
#include "ros/ros.h"

class InverseDynamics : Robot
{
private:
  double g;  // Acceleration due to gravity

  Eigen::Matrix<double, 1, 8> mass;               // List of link masses
  Eigen::Matrix<double, 3, 8> Pc;                 // Positions of link Centers of mass
  std::map<int, Eigen::Matrix<double, 3, 3>> Ic;  // Map of moments of inertia matrices

  std::map<int, Eigen::Matrix3d> R;  // Map of rotation matrices
  std::map<int, Eigen::Vector3d> P;  // Map of position vectors

  Eigen::Matrix<double, 1, 6> tau;    // Vector of Joint torques
  Eigen::Matrix<double, 3, 8> w;      // Link rotational displacements
  Eigen::Matrix<double, 3, 8> wdot;   // Link rotational velocities
  Eigen::Matrix<double, 3, 8> vdot;   // Link velocities
  Eigen::Matrix<double, 3, 8> vcdot;  // Link centers of mass velocities

  Eigen::Matrix<double, 3, 8> f;  // Dynamic forces
  Eigen::Matrix<double, 3, 8> n;  // Dynamics torques
  Eigen::Matrix<double, 3, 8> F;  // Static forces
  Eigen::Matrix<double, 3, 8> N;  // Static torques

  ros::Publisher inv_dyn_pub;

  template <typename Derived>
  void InitKinematics(const Eigen::MatrixBase<Derived>& q);  // Initializes kinematics by populating transformations
                                                             // matrices
  void InitDynamics();  // Initializes dynamics by populating mass,positions of ceters of mass, and moments of inertia
                        // values

  template <typename Derived>
  std::vector<double> CalcDynamics(const Eigen::MatrixBase<Derived>& q, const Eigen::MatrixBase<Derived>& qdot,
                                   const Eigen::MatrixBase<Derived>& qddot);  // calculates the dynamics

public:
  InverseDynamics(const ros::NodeHandle& nh, const ros::Publisher& pub);          // Constructor
  void InverseDynamicsCallback(const jasper_msgs::DynamicsInput::ConstPtr& msg);  // Callback
};
