#include <jasper_msgs/FKService.h>
#include <std_msgs/Float64.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <vector>

#include "ros/ros.h"

class ForwardKinematics
{
private:
  double dh_a2_, dh_a3_, dh_d1_, dh_d4_, dh_a6_, dh_dT_;

  std::vector<double> jointAnglesToEEPos(const std::vector<double>& jointAngle)
  {
    Eigen::Matrix<double, 4, 4> T;  // Transform matrix

    T << -sin(jointAngle[5]) *
                 (cos(jointAngle[3]) * sin(jointAngle[0]) +
                  sin(jointAngle[3]) * (cos(jointAngle[0]) * sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) -
                                        cos(jointAngle[0]) * cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2))) -
             cos(jointAngle[5]) *
                 (cos(jointAngle[4]) *
                      (sin(jointAngle[0]) * sin(jointAngle[3]) -
                       cos(jointAngle[3]) * (cos(jointAngle[0]) * sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) -
                                             cos(jointAngle[0]) * cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2))) -
                  sin(jointAngle[4]) * (cos(jointAngle[0]) * cos(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) +
                                        cos(jointAngle[0]) * cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[2]))),
        sin(jointAngle[5]) *
                (cos(jointAngle[4]) *
                     (sin(jointAngle[0]) * sin(jointAngle[3]) -
                      cos(jointAngle[3]) * (cos(jointAngle[0]) * sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) -
                                            cos(jointAngle[0]) * cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2))) -
                 sin(jointAngle[4]) * (cos(jointAngle[0]) * cos(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) +
                                       cos(jointAngle[0]) * cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[2]))) -
            cos(jointAngle[5]) *
                (cos(jointAngle[3]) * sin(jointAngle[0]) +
                 sin(jointAngle[3]) * (cos(jointAngle[0]) * sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) -
                                       cos(jointAngle[0]) * cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2))),
        -sin(jointAngle[4]) *
                (sin(jointAngle[0]) * sin(jointAngle[3]) -
                 cos(jointAngle[3]) * (cos(jointAngle[0]) * sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) -
                                       cos(jointAngle[0]) * cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2))) -
            cos(jointAngle[4]) * (cos(jointAngle[0]) * cos(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) +
                                  cos(jointAngle[0]) * cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[2])),
        dh_a2_ * cos(jointAngle[0]) * cos(jointAngle[1] - M_PI / 2) -
            dh_d4_ * (cos(jointAngle[0]) * cos(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) +
                      cos(jointAngle[0]) * cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[2])) -
            dh_dT_ *
                (sin(jointAngle[4]) *
                     (sin(jointAngle[0]) * sin(jointAngle[3]) -
                      cos(jointAngle[3]) * (cos(jointAngle[0]) * sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) -
                                            cos(jointAngle[0]) * cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2))) +
                 cos(jointAngle[4]) * (cos(jointAngle[0]) * cos(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) +
                                       cos(jointAngle[0]) * cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[2]))) -
            dh_a6_ *
                (sin(jointAngle[5]) *
                     (cos(jointAngle[3]) * sin(jointAngle[0]) +
                      sin(jointAngle[3]) * (cos(jointAngle[0]) * sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) -
                                            cos(jointAngle[0]) * cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2))) +
                 cos(jointAngle[5]) *
                     (cos(jointAngle[4]) *
                          (sin(jointAngle[0]) * sin(jointAngle[3]) -
                           cos(jointAngle[3]) *
                               (cos(jointAngle[0]) * sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) -
                                cos(jointAngle[0]) * cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2))) -
                      sin(jointAngle[4]) * (cos(jointAngle[0]) * cos(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) +
                                            cos(jointAngle[0]) * cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[2])))) -
            dh_a3_ * (cos(jointAngle[0]) * sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) -
                      cos(jointAngle[0]) * cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2)),
        sin(jointAngle[5]) *
                (cos(jointAngle[0]) * cos(jointAngle[3]) +
                 sin(jointAngle[3]) * (cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[0]) -
                                       sin(jointAngle[0]) * sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2))) +
            cos(jointAngle[5]) *
                (cos(jointAngle[4]) *
                     (cos(jointAngle[0]) * sin(jointAngle[3]) -
                      cos(jointAngle[3]) * (cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[0]) -
                                            sin(jointAngle[0]) * sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2))) +
                 sin(jointAngle[4]) * (cos(jointAngle[2]) * sin(jointAngle[0]) * sin(jointAngle[1] - M_PI / 2) +
                                       cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[0]) * sin(jointAngle[2]))),
        cos(jointAngle[5]) *
                (cos(jointAngle[0]) * cos(jointAngle[3]) +
                 sin(jointAngle[3]) * (cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[0]) -
                                       sin(jointAngle[0]) * sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2))) -
            sin(jointAngle[5]) *
                (cos(jointAngle[4]) *
                     (cos(jointAngle[0]) * sin(jointAngle[3]) -
                      cos(jointAngle[3]) * (cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[0]) -
                                            sin(jointAngle[0]) * sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2))) +
                 sin(jointAngle[4]) * (cos(jointAngle[2]) * sin(jointAngle[0]) * sin(jointAngle[1] - M_PI / 2) +
                                       cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[0]) * sin(jointAngle[2]))),
        sin(jointAngle[4]) *
                (cos(jointAngle[0]) * sin(jointAngle[3]) -
                 cos(jointAngle[3]) * (cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[0]) -
                                       sin(jointAngle[0]) * sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2))) -
            cos(jointAngle[4]) * (cos(jointAngle[2]) * sin(jointAngle[0]) * sin(jointAngle[1] - M_PI / 2) +
                                  cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[0]) * sin(jointAngle[2])),
        dh_a3_ * (cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[0]) -
                  sin(jointAngle[0]) * sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2)) -
            dh_d4_ * (cos(jointAngle[2]) * sin(jointAngle[0]) * sin(jointAngle[1] - M_PI / 2) +
                      cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[0]) * sin(jointAngle[2])) +
            dh_dT_ *
                (sin(jointAngle[4]) *
                     (cos(jointAngle[0]) * sin(jointAngle[3]) -
                      cos(jointAngle[3]) * (cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[0]) -
                                            sin(jointAngle[0]) * sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2))) -
                 cos(jointAngle[4]) * (cos(jointAngle[2]) * sin(jointAngle[0]) * sin(jointAngle[1] - M_PI / 2) +
                                       cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[0]) * sin(jointAngle[2]))) +
            dh_a6_ *
                (sin(jointAngle[5]) *
                     (cos(jointAngle[0]) * cos(jointAngle[3]) +
                      sin(jointAngle[3]) * (cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[0]) -
                                            sin(jointAngle[0]) * sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2))) +
                 cos(jointAngle[5]) *
                     (cos(jointAngle[4]) *
                          (cos(jointAngle[0]) * sin(jointAngle[3]) -
                           cos(jointAngle[3]) *
                               (cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[0]) -
                                sin(jointAngle[0]) * sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2))) +
                      sin(jointAngle[4]) * (cos(jointAngle[2]) * sin(jointAngle[0]) * sin(jointAngle[1] - M_PI / 2) +
                                            cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[0]) * sin(jointAngle[2])))) +
            dh_a2_ * cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[0]),
        cos(jointAngle[5]) * (sin(jointAngle[4]) * (cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2) -
                                                    sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2)) +
                              cos(jointAngle[3]) * cos(jointAngle[4]) *
                                  (cos(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) +
                                   cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[2]))) -
            sin(jointAngle[3]) * sin(jointAngle[5]) *
                (cos(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) +
                 cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[2])),
        -sin(jointAngle[5]) * (sin(jointAngle[4]) * (cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2) -
                                                     sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2)) +
                               cos(jointAngle[3]) * cos(jointAngle[4]) *
                                   (cos(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) +
                                    cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[2]))) -
            cos(jointAngle[5]) * sin(jointAngle[3]) *
                (cos(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) +
                 cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[2])),
        cos(jointAngle[3]) * sin(jointAngle[4]) *
                (cos(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) +
                 cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[2])) -
            cos(jointAngle[4]) * (cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2) -
                                  sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2)),
        dh_d1_ +
            dh_a6_ * (cos(jointAngle[5]) * (sin(jointAngle[4]) * (cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2) -
                                                                  sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2)) +
                                            cos(jointAngle[3]) * cos(jointAngle[4]) *
                                                (cos(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) +
                                                 cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[2]))) -
                      sin(jointAngle[3]) * sin(jointAngle[5]) *
                          (cos(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) +
                           cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[2]))) -
            dh_a3_ * (cos(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) +
                      cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[2])) -
            dh_d4_ * (cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2) -
                      sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2)) -
            dh_a2_ * sin(jointAngle[1] - M_PI / 2) -
            dh_dT_ * (cos(jointAngle[4]) * (cos(jointAngle[2]) * cos(jointAngle[1] - M_PI / 2) -
                                            sin(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2)) -
                      cos(jointAngle[3]) * sin(jointAngle[4]) *
                          (cos(jointAngle[2]) * sin(jointAngle[1] - M_PI / 2) +
                           cos(jointAngle[1] - M_PI / 2) * sin(jointAngle[2]))),
        0, 0, 0, 1;

    Eigen::Matrix3d rotm = T.block<3, 3>(0, 0);

    Eigen::Vector3d euler = rotm.eulerAngles(0, 1, 2);
    Eigen::Vector3d pos = T.block<3, 1>(0, 3);

    std::vector<double> result;

    for (int i = 0; i < 3; ++i)
    {
      result.push_back(pos(i));
    }
    for (int i = 0; i < 3; ++i)
    {
      result.push_back(euler(i));
    }

    return result;
  }

public:
  ForwardKinematics(const ros::NodeHandle& nodeH)
  {
    nodeH.getParam("dh_a2", dh_a2_);
    nodeH.getParam("dh_a3", dh_a3_);
    nodeH.getParam("dh_d1", dh_d1_);
    nodeH.getParam("dh_d4", dh_d4_);
    nodeH.getParam("dh_a6", dh_a6_);
    nodeH.getParam("dh_dT", dh_dT_);
  }

  bool FK_Callback(jasper_msgs::FKService::Request& req, jasper_msgs::FKService::Response& res)
  {
    std::vector<double> jointAngles = req.jointAngles;

    std::vector<double> eePos = jointAnglesToEEPos(jointAngles);

    res.eePos = eePos;

    std::string request = "Received : ";
    std::string response = "Response : ";

    for (int i = 0; i < jointAngles.size(); ++i)
    {
      request += std::to_string(jointAngles[i]);
      request += " ";
    }

    for (int i = 0; i < eePos.size(); ++i)
    {
      response += std::to_string(eePos[i]);
      response += " ";
    }

    ROS_INFO("%s", request.c_str());
    ROS_INFO("%s", response.c_str());

    return true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fk_server");

  ros::NodeHandle nh;

  ForwardKinematics forwardKinematics(nh);

  ros::ServiceServer fk_server =
      nh.advertiseService("forward_kinematics_service", &ForwardKinematics::FK_Callback, &forwardKinematics);

  ros::spin();

  return 0;
}
