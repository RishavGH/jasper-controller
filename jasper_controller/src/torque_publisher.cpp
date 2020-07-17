#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

using namespace std;

class TorquePublisher
{
	public:
	TorquePublisher(const ros::Publisher& pub1, const ros::Publisher& pub2, const ros::Publisher& pub3, const ros::Publisher& pub4,
		const ros::Publisher& pub5, const ros::Publisher& pub6): desired_angle_(6,0)
	{
		tor_pub1_ = pub1;
		tor_pub2_ = pub2;
		tor_pub3_ = pub3;
		tor_pub4_ = pub4;
		tor_pub5_ = pub5;
		tor_pub6_ = pub6;
	}
	
	void torque_callback(const sensor_msgs::JointState::ConstPtr& msg)
	{
		vector<double> joint_angle = msg->position;
		vector<double> joint_vel = msg->velocity;
		//float joint1_angle = msg->position[0];
		//float joint1_vel = msg->velocity[0];
		
		//float torque = Kp_ * (desired_angle_ - joint1_angle) + Kd_ * (0 - joint1_vel);
		
		vector<double> torque(6,0);
		
		for (int i=0; i<6; ++i)
		{
			torque[i] = Kp_[i] * (desired_angle_[i] - joint_angle[i]) + Kd_[i] * (0 - joint_vel[i]);
		}
		
		std_msgs::Float64 torque1_msg;
		std_msgs::Float64 torque2_msg;
		std_msgs::Float64 torque3_msg;
		std_msgs::Float64 torque4_msg;
		std_msgs::Float64 torque5_msg;
		std_msgs::Float64 torque6_msg;
		
		torque1_msg.data = torque[0];
		torque2_msg.data = torque[1];
		torque3_msg.data = torque[2];
		torque4_msg.data = torque[3];
		torque5_msg.data = torque[4];
		torque6_msg.data = torque[5];
		
		tor_pub1_.publish(torque1_msg);
		tor_pub2_.publish(torque2_msg);
		tor_pub3_.publish(torque3_msg);
		tor_pub4_.publish(torque4_msg);
		tor_pub5_.publish(torque5_msg);
		tor_pub6_.publish(torque6_msg);
	}
	
	void desired_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
	{
		desired_angle_ = msg->data;
		
		cout<< "Received : "<<desired_angle_[0]<<" "<<desired_angle_[1]<<" "<<desired_angle_[3]<<" "<<desired_angle_[4]
			<<" "<<desired_angle_[5]<<" "<<desired_angle_[6]<<endl;
	}
	
	private:
	vector<double> desired_angle_;
	vector<double> Kp_{100, 100, 100, 20, 20, 10};
	vector<double> Kd_{10, 10, 10, 5, 5, 5};
	
	//float Kp_ = 100, Kd_ = 10, Ki_ = 5;
	ros::Publisher tor_pub1_, tor_pub2_, tor_pub3_, tor_pub4_, tor_pub5_, tor_pub6_;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "torque_publisher");
	
	ros::NodeHandle nh;
	
	ros::Publisher pub1 = nh.advertise<std_msgs::Float64>("joint1_controller/command", 10, true);
	ros::Publisher pub2 = nh.advertise<std_msgs::Float64>("joint2_controller/command", 10, true);
	ros::Publisher pub3 = nh.advertise<std_msgs::Float64>("joint3_controller/command", 10, true);
	ros::Publisher pub4 = nh.advertise<std_msgs::Float64>("joint4_controller/command", 10, true);
	ros::Publisher pub5 = nh.advertise<std_msgs::Float64>("joint5_controller/command", 10, true);
	ros::Publisher pub6 = nh.advertise<std_msgs::Float64>("joint6_controller/command", 10, true);
	
	TorquePublisher tpub(pub1, pub2, pub3, pub4, pub5, pub6);
	
	ros::Subscriber torq_sub = nh.subscribe("joint_states", 10, &TorquePublisher::torque_callback, &tpub);
	
	ros::Subscriber angle_sub = nh.subscribe("desired_angle", 10, &TorquePublisher::desired_callback, &tpub);
	
	ros::Rate loop_rate(100);
	
	ros::spin();
	
/*	std_msgs::Float64 msg;
	
	double torques[] = {-0.1, -0.1};
	double torque = 0.0;
	
	int i =0;
	while(ros::ok())
	{
		cout<<"Enter torque: ";
		cin>>torque;
		
		msg.data = torque;
		ROS_INFO("Moving.");
		pub.publish(msg);
	
		ros::spinOnce();
		
		loop_rate.sleep();
		++i;
	}*/
	
/*	msg.data = 0.0f;
	
	ROS_INFO("Stopped.");
	
	pub.publish(msg);*/
	
	return 0;
}
