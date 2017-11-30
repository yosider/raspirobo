#!/usr/bin/env python
# coding: utf-8

import numpy as np

import rospy
from raspirobo.msg import Floats
from rospy.numpy_msg import numpy_msg

from servo_utils import Servos


class Body():
    ''' Subscribe servo signals and set them. '''
    def __init__(self):
	print("body initialization done!")
        self.servos = Servos()
        self.subscriber = rospy.Subscriber('/raspirobo/servo_signals', numpy_msg(Floats), self.callback, queue_size=1, buff_size=2**24)

    def callback(self, msg):
        signals = msg.data
        if len(signals)>0:
            print "data type:", type(signals[0])
            necks = 250 + signals[:2] * 50  # in [250, 300]
            necks = np.clip(necks, 250, 300)
            necks = list(map(int, necks))
            print "necks:", necks
            legs = 176 + signals[2:8] * 200  # in [176, 376]
            legs = list(map(int, legs))
            legs = np.clip(legs, 176, 376)
            print "legs:", legs
            self.servos.setPCA9685Duty2(6, *necks)
            self.servos.setPCA9685Duty6(0, *legs)


if __name__ == '__main__':
    node = Body()
    rospy.init_node('body', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down node body")
