#!/usr/bin/env python

import rospy
from us_digital_encoders.msg import USDigitalEncoders
from std_msgs.msg import Float64

class EncoderInterface(object):

    def __init__(self, ticks_per_metre, ticks_per_degree):
        rospy.init_node('encoder_interface')
        self.ticks_per_metre = ticks_per_metre
	self.ticks_per_degree = ticks_per_degree
        self.last_msg_time = None
        self.sub = rospy.Subscriber('encoders', USDigitalEncoders, self._callback)
        self.pub_speed = rospy.Publisher('throttle/state/data', Float64, queue_size=5)
        self.pub_direction = rospy.Publisher('steering/state/data', Float64, queue_size=5)

    def _callback(self, msg):
	speed_ticks = float(msg.ticks[0])
	direction_ticks = float(msg.ticks[1])
        timestamp = msg.header.stamp.to_sec()
        if self.last_msg_time is None:
            self.last_msg_time = timestamp
        else:
            speed = self._ticks_to_speed(speed_ticks, timestamp)
            self.pub_speed.publish(speed)
            direction = self._ticks_to_direction(direction_ticks)
            self.pub_direction.publish(direction)
            self.last_msg_time = timestamp
    
    def _ticks_to_speed(self, ticks, timestamp):
        delta_metres = ticks / self.ticks_per_metre
        speed = delta_metres / (timestamp - self.last_msg_time)
        return speed

    def _ticks_to_direction(self, ticks):
        direction = ticks * self.ticks_per_degree
        return direction


if __name__ == '__main__':
    encoder_interface = EncoderInterface(47363, 100)
    rospy.spin()
