#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Empty


class CarReset(object):

	def __init__(self):
		rospy.init_node('reset_node', anonymous=False)
		self.sub = rospy.Subscriber("reset", Empty, self._callback)
		self.pub_speed = rospy.Publisher('throttle/setpoint', Float64, queue_size=5)
		self.pub_direction = rospy.Publisher('steering/setpoint', Float64, queue_size=5)

	def _callback(self, msg):
		rospy.logwarn(rospy.get_caller_id() + ": Controller Setpoints Reset to 0")
		self.pub_speed.publish(0.0)
		self.pub_direction.publish(0.0)		

if __name__ == '__main__':
	car_reset = CarReset()
	rospy.spin()	


