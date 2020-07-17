#!/usr/bin/env/python

import rospy
from std_msgs.msg import String

def talker():
	pub = rospy.Publisher('chatterPy', String, queue_size=100)
	
	rospy.init_node('talkerPy', anonymous=True)
	
	rate = rospy.Rate(10)
	
	count = 1
	while not rospy.is_shutdown():
		test_str = 'Test Code Py : %d' % count
		rospy.loginfo(test_str)
		
		pub.publish(test_str)
		count += 1
		
		rate.sleep()
		
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
