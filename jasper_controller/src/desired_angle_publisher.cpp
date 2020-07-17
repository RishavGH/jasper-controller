#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "desired_angle_publisher");
	
	ros::NodeHandle nh;
	
	ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("desired_angle", 10);
	
	vector<float> input(6,0);
	std_msgs::Float64MultiArray msg;
	
	while(ros::ok())
	{
		msg.data.clear();
		
		cout<<"Enter desired angles in radians(-20 to exit) : ";
		cin>>input[0]>>input[1]>>input[2]>>input[3]>>input[4]>>input[5];
		
		/*if(input == -20)
			break;*/
		
		for (int i=0; i<6; ++i)
			msg.data.push_back(input[i]);
		
		pub.publish(msg);
		
		ros::spinOnce();
	}
	
	return 0;
}
