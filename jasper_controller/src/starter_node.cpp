/********************************************************************************/
/*********************JASPER CONTROLLER DEMO STARTER NODE************************
 * Node for demonstrating the jasper controller over a predefined trajectory for
 * an assistive feeding task.
 *********************************************************************************/
#define EIGEN_STACK_ALLOCATION_LIMIT 0

#include <cmath>
#include <csignal>
#include <iostream>

#include "jasper_controller/starter.h"
#include "jasper_msgs/IKService.h"
#include "jasper_msgs/JointInfo.h"
#include "std_srvs/Empty.h"

// To signal shutdown
sig_atomic_t volatile shutdown_key = 0;

void starter_sigint_handler(int signum)
{
  shutdown_key = 1;
}

Starter::Starter(const ros::Publisher& starter_pub) : pub(starter_pub)
{
  // base2bowl << 0.6, 0.2, 0.003, 0, 0, 0;
  base2bowl << 0.5, 0.0, 0.003, 0, 0, 0;
  base2mouth << 0.6, 0.36, 0.35, 0, 0, 0;

  waypoints.block<6, 1>(0, 0) << 0.5089, 0, 0.584, 0, 2.0071, -0.7854;

  Eigen::Matrix<double, 6, 4> relPoints;
  relPoints.col(0) << 0, -0.05, 0.040, 0, 2.0071, -0.7854;
  relPoints.col(1) << 0, 0, 0.030, -2.5457, 1.0349, 2.4738;
  relPoints.col(2) << 0, 0.04, 0.065, -1.5708, 1.1903, 1.5708;
  relPoints.col(3) << 0, 0.035, 0.068, -1.5708, 1.1975, 1.5708;

  Eigen::Matrix<double, 6, 2> mouthPoints;
  mouthPoints.col(0) << 0, -0.1, -0.01, -1.5708, 1.1619, 1.5708;
  mouthPoints.col(1) << 0, 0, 0, -1.5708, 0.3490, 1.5708;

  for (int i = 0; i < 4; ++i)
  {
    relPoints.col(i) = base2bowl + relPoints.col(i);
  }

  for (int i = 0; i < 2; ++i)
  {
    mouthPoints.col(i) = base2mouth + mouthPoints.col(i);
  }

  waypoints.block<6, 4>(0, 1) = relPoints;
  waypoints.block<6, 2>(0, 5) = mouthPoints;

  timepoints << 0, 2, 2.5, 3.2, 3.4, 5.4, 7;
}

void Starter::InitJointPoints(ros::ServiceClient& client)
{
  jasper_msgs::IKServiceRequest srv_req;
  jasper_msgs::IKServiceResponse srv_res;

  std::vector<double> initializer(6, 0);

  for (int i = 0; i < 7; ++i)
  {
    srv_req.eePos = matrixToStdVector(waypoints.block<6, 1>(0, i));
    srv_req.lastPose = initializer;

    if (client.call(srv_req, srv_res))
    {
      jointpoints.col(i) = stdVectorToMatrix(srv_res.jointAngles);
    }
    else
    {
      ROS_ERROR("Failed to call service inverse_kinematics_service!");
    }
  }

  for (int i = 1; i < 6; ++i)
  {
    wpVels.col(i) = (jointpoints.col(i) - jointpoints.col(i - 1)) / (timepoints(i) - timepoints(i - 1));
  }
  wpVels.col(0) = Eigen::Matrix<double, 6, 1>::Zero();
  wpVels.col(1) = Eigen::Matrix<double, 6, 1>::Zero();
  wpVels.col(6) = Eigen::Matrix<double, 6, 1>::Zero();
}

void Starter::createTraj()
{
  Eigen::Matrix<double, 1, 700> timeSamples;
  timeSamples = Eigen::Matrix<double, 1, 700>::LinSpaced(700, 0, 7);

  cubicpolytraj(jointpoints, wpVels, timepoints, timeSamples, trajectory, jointVels, jointAccels);
}

void Starter::PublishJointPoints(int iteration_step)
{
  std::vector<double> jointPose;
  std::vector<double> jointVel;
  std::vector<double> jointAccel;

  // To facilitate infinite iteration
  iteration_step = iteration_step > 698 ? 698 : iteration_step;

  // std::cout << "Joint Pose : " << trajectory.col(iteration_step) << std::endl;
  // std::cout << "Joint Vel : " << jointVels.col(iteration_step) << std::endl;
  // std::cout << "Joint Acc : " << jointAccels.col(iteration_step) << std::endl;

  jointPose = matrixToStdVector(trajectory.col(iteration_step));
  jointVel = matrixToStdVector(jointVels.col(iteration_step));
  jointAccel = matrixToStdVector(jointAccels.col(iteration_step));

  // jointPose = { 0.3, 0, 0, 0, 0, 0 };
  // jointVel = { 0, 0, 0, 0, 0, 0 };
  // jointAccel = { 0, 0, 0, 0, 0, 0 };

  jasper_msgs::JointInfo msg;
  msg.jointAngles = jointPose;
  msg.jointVelocities = jointVel;
  msg.jointAccel = jointAccel;

  msg.header.stamp = ros::Time::now();
  pub.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "starter_node", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh;

  // Registering SIGINT handler
  signal(SIGINT, starter_sigint_handler);

  ros::Publisher pub = nh.advertise<jasper_msgs::JointInfo>("joint_input", 10);
  ros::ServiceClient client = nh.serviceClient<jasper_msgs::IKService>("inverse_kinematics_service");

  float timeout = 5.0f;

  if (!client.waitForExistence(ros::Duration(timeout)))
  {
    ROS_ERROR("Call to \"%s\" service timed out [%f sec].", client.getService().c_str(), timeout);
  }

  Starter starter(pub);
  starter.InitJointPoints(client);
  // starter.cubicpolytraj();
  starter.createTraj();

  // To pause physics
  ros::ServiceClient pausePhysics = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");

  if (!pausePhysics.waitForExistence(ros::Duration(timeout)))
  {
    ROS_ERROR("Call to \"%s\" service timed out [%f sec].", pausePhysics.getService().c_str(), timeout);
    std::cout << "Cannot pause physics" << std::endl;
  }

  // To unpause physics
  ros::ServiceClient unpausePhysics = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

  if (!unpausePhysics.waitForExistence(ros::Duration(timeout)))
  {
    ROS_ERROR("Call to \"%s\" service timed out [%f sec].", unpausePhysics.getService().c_str(), timeout);
    std::cout << "Cannot unpause physics" << std::endl;
  }

  ros::Rate rate(100);

  std::cout << "Press enter to enable physics and start simulation : ";
  std::cin.get();

  std_srvs::Empty empty_req;

  unpausePhysics.call(empty_req);

  std::cout << "Physics unpaused!" << std::endl;

  // std::cout << "Press enter to begin...";
  // std::cin.get();

  int i = 0;

  while (!shutdown_key)
  {
    starter.PublishJointPoints(i);

    ros::spinOnce();

    rate.sleep();
    ++i;
  }

  // To pause physics when shutting down
  pausePhysics.call(empty_req);

  ros::shutdown();

  return 0;
}
