#include <jasper_msgs/IKService.h>
#include <std_msgs/Float64.h>

#include <iostream>
#include <map>
#include <vector>

#include "inverse_kinematics/ik_math.h"
#include "ros/ros.h"

class InverseKinematics
{
public:
  InverseKinematics(const std::vector<double>& dh_params) : ik_math(dh_params)
  {
    dh_a2_ = dh_params[0];
    dh_a3_ = dh_params[1];
    dh_d4_ = dh_params[2];
    dh_a6_ = dh_params[3];
    dh_dT_ = dh_params[4];
  }

  bool IK_Callback(jasper_msgs::IKService::Request& req, jasper_msgs::IKService::Response& res)
  {
    std::vector<double> position = req.eePos;
    std::vector<double> lastPose = req.lastPose;

    res.jointAngles = ik_math.IRB120_IK(position, lastPose);

    ROS_INFO("Service Response: %.4f %.4f %.4f %.4f %.4f %.4f", res.jointAngles[0], res.jointAngles[1],
             res.jointAngles[2], res.jointAngles[3], res.jointAngles[4], res.jointAngles[5]);

    return true;
  }

private:
  double dh_a2_, dh_a3_, dh_d4_, dh_a6_, dh_dT_;
  IK_Math ik_math;
};

void setDHParameters(ros::NodeHandle& nodeH, std::vector<double>& dh_params)
{
  double a2, a3, d4, a6, dT;

  while (!nodeH.hasParam("dh_a2") || !nodeH.hasParam("dh_a3") || !nodeH.hasParam("dh_d4") || !nodeH.hasParam("dh_a6") ||
         !nodeH.hasParam("dh_dT") || !nodeH.hasParam("dh_dT"))
  {
  }

  nodeH.getParam("dh_a2", a2);
  nodeH.getParam("dh_a3", a3);
  nodeH.getParam("dh_d4", d4);
  nodeH.getParam("dh_a6", a6);
  nodeH.getParam("dh_dT", dT);

  dh_params.push_back(a2);
  dh_params.push_back(a3);
  dh_params.push_back(d4);
  dh_params.push_back(a6);
  dh_params.push_back(dT);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ik_server");
  ros::NodeHandle nh;

  std::vector<double> dh_params;

  setDHParameters(nh, dh_params);
  InverseKinematics ik(dh_params);

  ros::ServiceServer ik_serv = nh.advertiseService("inverse_kinematics_service", &InverseKinematics::IK_Callback, &ik);

  ros::spin();

  return 0;
}
