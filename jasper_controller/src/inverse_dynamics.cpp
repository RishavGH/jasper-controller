#include <cmath>
#include <iostream>
#include <string>

#include "jasper_controller/inverse_dynamics.h"

InverseDynamics::InverseDynamics(const ros::NodeHandle& nh, const ros::Publisher& pub)
  : tau(Eigen::Matrix<double, 1, 6>::Zero())
  , w(Eigen::Matrix<double, 3, 8>::Zero())
  , wdot(Eigen::Matrix<double, 3, 8>::Zero())
  , vdot(Eigen::Matrix<double, 3, 8>::Zero())
  , vcdot(Eigen::Matrix<double, 3, 8>::Zero())
  , f(Eigen::Matrix<double, 3, 8>::Zero())
  , n(Eigen::Matrix<double, 3, 8>::Zero())
  , F(Eigen::Matrix<double, 3, 8>::Zero())
  , N(Eigen::Matrix<double, 3, 8>::Zero())
  , inv_dyn_pub(pub)
  , Robot(nh)
{
  InitDynamics();
}

void InverseDynamics::InitDynamics()
{
  // Link 1 parameters

  double m1 = 3.067;

  double Ic1x = 0.0223798;
  double Ic1y = 0.0225665;
  double Ic1z = 0.0104534;
  double Ic1xy = -0.0000129;
  double Ic1xz = -0.0000386;
  double Ic1zy = 0.0000382;

  double lc1x = 0.0000977;
  double lc1y = -0.0001193;
  double lc1z = -0.0515882;

  // Link 2 parameters

  double m2 = 3.90867;

  double Ic2x = 0.0259893;
  double Ic2y = 0.1004078;
  double Ic2z = 0.0816359;
  double Ic2xy = 0.0003652;
  double Ic2xz = -0.0016395;
  double Ic2zy = 0.0000011;

  double lc2x = 0.1012432;
  double lc2y = 0.0007783;
  double lc2z = -0.0028670;

  // Link 3 parameters

  double m3 = 2.94372;

  double Ic3x = 0.0142330;
  double Ic3y = 0.0182316;
  double Ic3z = 0.0281164;
  double Ic3xy = 0.0053169;
  double Ic3xz = -0.0000008;
  double Ic3zy = -0.0000087;

  double lc3x = 0.0579106;
  double lc3y = 0.0228077;
  double lc3z = 0.0010644;

  // Link 4 parameters

  double m4 = 1.32509;

  double Ic4x = 0.0131807;
  double Ic4y = 0.0119401;
  double Ic4z = 0.0028397;
  double Ic4xy = -0.0000132;
  double Ic4xz = -0.0000581;
  double Ic4zy = 0.0000402;

  double lc4x = 0.0004064;
  double lc4y = -0.0002253;
  double lc4z = -0.0773884;

  // Link 5 parameters

  double m5 = 0.54663;

  double Ic5x = 0.0008161;
  double Ic5y = 0.0004049;
  double Ic5z = 0.0008935;
  double Ic5xy = 0.0000008;
  double Ic5xz = 0;
  double Ic5zy = 0.0000016;

  double lc5x = 0.0000622;
  double lc5y = -0.0010948;
  double lc5z = 0.0000369;

  // Link 6 parameters

  double m6 = 0.01368;

  double Ic6x = 0.0000594;
  double Ic6y = 0.0000593;
  double Ic6z = 0.0000030;
  double Ic6xy = 0;
  double Ic6xz = -0.0000002;
  double Ic6zy = 0;

  double lc6x = -0.0001696;
  double lc6y = -0.0000013;
  double lc6z = 0.0649379;

  // Link 7 parameters

  double m7 = 0.01496;

  double Ic7x = 0.0001567;
  double Ic7y = 0.0001761;
  double Ic7z = 0.0000224;
  double Ic7xy = 0;
  double Ic7xz = 0.0000547;
  double Ic7zy = 0;

  double lc7x = -0.0328898;
  double lc7y = -0.0000093;
  double lc7z = -0.0826049;

  mass << 0, m1, m2, m3, m4, m5, m6, m7;

  Pc << 0, lc1x, lc2x, lc3x, lc4x, lc5x, lc6x, lc7x, 0, lc1y, lc2y, lc3y, lc4y, lc5y, lc6y, lc7y, 0, lc1z, lc2z, lc3z,
      lc4z, lc5z, lc6z, lc7z;

  Ic[0] = Eigen::Matrix<double, 3, 3>::Zero();

  Ic[1] << Ic1x, Ic1xy, Ic1xz, Ic1xy, Ic1y, Ic1zy, Ic1xz, Ic1zy, Ic1z;

  Ic[2] << Ic2x, Ic2xy, Ic2xz, Ic2xy, Ic2y, Ic2zy, Ic2xz, Ic2zy, Ic2z;

  Ic[3] << Ic3x, Ic3xy, Ic3xz, Ic3xy, Ic3y, Ic3zy, Ic3xz, Ic3zy, Ic3z;

  Ic[4] << Ic4x, Ic4xy, Ic4xz, Ic4xy, Ic4y, Ic4zy, Ic4xz, Ic4zy, Ic4z;

  Ic[5] << Ic5x, Ic5xy, Ic5xz, Ic5xy, Ic5y, Ic5zy, Ic5xz, Ic5zy, Ic5z;

  Ic[6] << Ic6x, Ic6xy, Ic6xz, Ic6xy, Ic6y, Ic6zy, Ic6xz, Ic6zy, Ic6z;

  Ic[7] << Ic7x, Ic7xy, Ic7xz, Ic7xy, Ic7y, Ic7zy, Ic7xz, Ic7zy, Ic7z;

  g = 9.81;
  // g = 0.0;

  vdot.block<3, 1>(0, 0) << 0, 0, g;
}

template <typename Derived>
void InverseDynamics::InitKinematics(const Eigen::MatrixBase<Derived>& q)
{
  // T: 0(Base) -> 0

  R[0] = Eigen::Matrix3d::Zero();
  P[0] = Eigen::Vector3d::Zero();

  // T: 0(Base) -> 1

  R[1] << cos(q(0)), -sin(q(0)), 0, sin(q(0)), cos(q(0)), 0, 0, 0, 1;
  P[1] << 0, 0, dh_d1_;

  // T: 1 -> 2

  R[2] << cos(q(1) - M_PI / 2), -sin(q(1) - M_PI / 2), 0, 0, 0, 1, -sin(q(1) - M_PI / 2), -cos(q(1) - M_PI / 2), 0;
  P[2] << 0, 0, 0;

  // T: 2 -> 3

  R[3] << cos(q(2)), -sin(q(2)), 0, sin(q(2)), cos(q(2)), 0, 0, 0, 1;
  P[3] << dh_a2_, 0, 0;

  // T: 3 -> 4

  R[4] << cos(q(3)), -sin(q(3)), 0, 0, 0, 1, -sin(q(3)), -cos(q(3)), 0;
  P[4] << dh_a3_, dh_d4_, 0;

  // T: 4 -> 5

  R[5] << cos(q(4)), -sin(q(4)), 0, 0, 0, -1, sin(q(4)), cos(q(4)), 0;
  P[5] << 0, 0, 0;

  // T: 5 -> 6

  R[6] << cos(q(5) + M_PI), -sin(q(5) + M_PI), 0, 0, 0, 1, -sin(q(5) + M_PI), -cos(q(5) + M_PI), 0;
  P[6] << 0, 0, 0;

  // T: 6 -> 7(End Effector)

  R[7] = Eigen::Matrix3d::Identity();
  P[7] << dh_a6_, 0, dh_dT_;
}

template <typename Derived>
std::vector<double> InverseDynamics::CalcDynamics(const Eigen::MatrixBase<Derived>& q,
                                                  const Eigen::MatrixBase<Derived>& qdot,
                                                  const Eigen::MatrixBase<Derived>& qddot)
{
  InitKinematics(q);

  Eigen::Matrix<double, 1, 8> thetadot = Eigen::Matrix<double, 1, 8>::Zero();
  thetadot.block<1, 6>(0, 1) = qdot;

  Eigen::Matrix<double, 1, 8> thetaddot = Eigen::Matrix<double, 1, 8>::Zero();
  thetaddot.block<1, 6>(0, 1) = qddot;

  Eigen::Vector3d zAxisVector;
  zAxisVector << 0, 0, 1;

  // Outward Iteration

  for (int i = 0; i < 7; ++i)
  {
    w.block<3, 1>(0, i + 1) = (R[i + 1].transpose() * w.block<3, 1>(0, i)) + zAxisVector * thetadot(i + 1);
    wdot.block<3, 1>(0, i + 1) = (R[i + 1].transpose() * wdot.block<3, 1>(0, i)) +
                                 (R[i + 1].transpose() * w.block<3, 1>(0, i)).cross(zAxisVector * thetadot(i + 1)) +
                                 zAxisVector * thetaddot(i + 1);

    vdot.block<3, 1>(0, i + 1) =
        R[i + 1].transpose() *
        (wdot.block<3, 1>(0, i).cross(P[i + 1]) + w.block<3, 1>(0, i).cross(w.block<3, 1>(0, i).cross(P[i + 1])) +
         vdot.block<3, 1>(0, i));

    vcdot.block<3, 1>(0, i + 1) =
        wdot.block<3, 1>(0, i + 1).cross(Pc.block<3, 1>(0, i + 1)) +
        w.block<3, 1>(0, i + 1).cross(w.block<3, 1>(0, i + 1).cross(Pc.block<3, 1>(0, i + 1))) +
        vdot.block<3, 1>(0, i + 1);

    F.block<3, 1>(0, i + 1) = mass(i + 1) * vcdot.block<3, 1>(0, i + 1);
    N.block<3, 1>(0, i + 1) =
        Ic[i + 1] * wdot.block<3, 1>(0, i + 1) + w.block<3, 1>(0, i + 1).cross(Ic[i + 1] * w.block<3, 1>(0, i + 1));
  }

  // Inward Iteration

  for (int i = 6; i > 0; --i)
  {
    f.block<3, 1>(0, i) = R[i + 1] * f.block<3, 1>(0, i + 1) + F.block<3, 1>(0, i);
    n.block<3, 1>(0, i) = N.block<3, 1>(0, i) + R[i + 1] * n.block<3, 1>(0, i + 1) +
                          Pc.block<3, 1>(0, i).cross(F.block<3, 1>(0, i)) +
                          P[i + 1].cross(R[i + 1] * f.block<3, 1>(0, i + 1));
  }

  tau = n.block<1, 6>(2, 1);

  std::vector<double> result;

  for (int i = 0; i < 6; ++i)
  {
    result.push_back(tau(i));
  }

  return result;
}

void InverseDynamics::InverseDynamicsCallback(const jasper_msgs::DynamicsInput::ConstPtr& msg)
{
  jasper_msgs::DynamicsOutput torque_command;

  std::vector<double> jointAngles = msg->jointAngles;
  std::vector<double> jointVelocities = msg->jointVelocities;
  std::vector<double> accel = msg->accel;

  Eigen::Matrix<double, 6, 1> q = stdVectorToMatrix(jointAngles);
  Eigen::Matrix<double, 6, 1> qdot = stdVectorToMatrix(jointVelocities);
  Eigen::Matrix<double, 6, 1> qddot = stdVectorToMatrix(accel);

  std::vector<double> torque_commands = CalcDynamics(q, qdot, qddot);

  jasper_msgs::DynamicsOutput response;
  response.jointTorques = torque_commands;

  std::string log_output("Torque Command : ");
  for (int i = 0; i < torque_commands.size(); ++i)
  {
    log_output += std::to_string(torque_commands[i]);
    log_output += " ";
  }

  // ROS_INFO("%s", log_output.c_str());

  inv_dyn_pub.publish(response);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "inverse_dynamics_node");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<jasper_msgs::DynamicsOutput>("torque_command", 10);

  InverseDynamics inverseDynamics(nh, pub);

  ros::Subscriber sub =
      nh.subscribe("resolved_command", 10, &InverseDynamics::InverseDynamicsCallback, &inverseDynamics);

  ros::spin();

  return 0;
}
