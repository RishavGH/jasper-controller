#include <ros/ros.h>
#include <string.h>
#include <std_msgs/String.h>

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "talkerC");
	
	ros::NodeHandle nh;
	
	ros::Publisher pub = nh.advertise<std_msgs::String>("chatterC", 10);
	
	ros::Rate loop_rate(10);
	
	string predicate = "Test Code C : ";
	string msg_string;
	
	std_msgs::String msg;
	
	int count = 1;
	while(ros::ok())
	{
		msg_string = predicate + to_string(count);
		
		msg.data = msg_string;
		ROS_INFO("%s", msg.data.c_str());
		
		pub.publish(msg);
		
		ros::spinOnce();
		
		loop_rate.sleep();
		++count;
	}

	
	return 0;
}
