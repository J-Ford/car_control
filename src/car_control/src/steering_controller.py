#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from us_digital_encoders.msg import USDigitalEncoders

#TODO: Calibrate these
STEERING_MAX_DEGREES = 30
ENCODER_MAX_TICKS = 6000



def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	#read encoder ticks message and convert to angle
		#ie data/ENCODER_MAX_TICKS * STEERING_MAX_DEGREES
	#send data to pid in form of float64


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('steering_controller', anonymous=False)

    rospy.Subscriber("set_direction", USDigitalEncoders, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
