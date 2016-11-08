#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
import random as rd

def talker():
	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=100)
	rospy.init_node('omap', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	msg = Twist()
	while not rospy.is_shutdown():
		msg.linear.x = rd.uniform(1,4)
		msg.angular.z = rd.uniform(1,4)
		rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
