import rospy
from std_msgs.msg import String

def listenerCallback(msg):
	rospy.loginfo(rospy.get_caller_id() + 'I heard ' + msg.data)
	
def listener():
	sub = rospy.Subscriber('chatterPy', String, listenerCallback)
	rospy.init_node('listenerPy', anonymous=True)
	
	rospy.spin()
	
if __name__=='__main__':
	listener()
