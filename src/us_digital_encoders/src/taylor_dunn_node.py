#!/usr/bin/env python

import rospy
from us_digital_encoders.msg import USDigitalEncoders
from driver import UsDigitalSei

class Encoders(object):

    def __init__(self):
        rospy.init_node('encoder_reader')
        self.max_count = 65535
        # initializations
        self.prev_data = None
        self.prev_drive_ticks = None # left ticks previous
        # subscribe to serial port, set up publisher
        self.encoders = UsDigitalSei(port='/dev/ttyUSB0', platform='GD')
        self.pub = rospy.Publisher('encoders', USDigitalEncoders, queue_size=5)

    def _generate_encoder_msg(self, delta_drive_ticks, steer_ticks):
        msg = USDigitalEncoders()
        msg.ticks = [delta_drive_ticks, steer_ticks]
        msg.header.stamp = rospy.Time.now()
        return msg

    def _handle_rollover(self, num):
        if num < -30000: # roll from 65535 -> 0
            num += self.max_count + 1
        elif num > 30000: # roll from 0 -> 65535
            num -= self.max_count - 1
        return num

    def _parse_data(self, data):
	rospy.loginfo("Data received: %s", data)
        drive, steer_ticks = data.split(',')
        drive_roll, drive_ticks = drive.split('-')
        return int(drive_ticks), int(steer_ticks)

    def run(self):
        while not rospy.is_shutdown():
            data = self.encoders.read()
            # store first data
            if self.prev_data is None:
                self.prev_data = data
                self.prev_drive_ticks, _ = self._parse_data(data)
                continue
            # get the current ticks of each encoder
            try:
                drive_ticks, steer_ticks = self._parse_data(data)
            except ValueError:
                continue
            # calculate the number of ticks since the drive encoder was last polled
            # NOTE: encoder count INCREASES if vehicle is driven BACKWARDS
            delta_drive_ticks = -(drive_ticks - self.prev_drive_ticks)
            # handle rollover
            delta_drive_ticks = self._handle_rollover(delta_drive_ticks)
            steer_ticks = self._handle_rollover(steer_ticks)
            # publish encoder data
            encoder_msg = self._generate_encoder_msg(delta_drive_ticks, steer_ticks)
            self.pub.publish(encoder_msg)
            # record previous drive encoder data
            self.prev_drive_ticks = drive_ticks

if __name__ == '__main__':
    enc = Encoders()
    enc.run()
