#!/usr/bin/env python
# coding: utf-8

import numpy as np

import rospy
from raspirobo.msg import Floats
from rospy.numpy_msg import numpy_msg

from build_models import build_action_decoder

class Action_decoder():
    ''' Subscribe: abstruct action signal from action_maker
        Publish: concrete servo action signal to actuator'''
    def __init__(self):
        self.subscriber = rospy.Subscriber('/raspirobo/action', numpy_msg(Floats), self.callback, queue_size=1, buff_size=2**24)
        self.publisher = rospy.Publisher('/raspirobo/servo_signals', numpy_msg(Floats), queue_size=10)
        self.model = build_action_decoder()
        self.model.predict(np.zeros([1,16]))

    def callback(self, msg):
        action = self.msg2action(msg)
        servo_sig = self.get_servo_signal(action)
        print "servo signal:", servo_sig
        self.publisher.publish(servo_sig)

    def msg2action(self, msg):
        assert msg.data.shape == (16,)
        action = np.expand_dims(msg.data, axis=0)
        return action

    def get_servo_signal(self, action):
        # output action element is model output, in [0., 1.]
        signal = self.model.predict(action)[0]
        signal = np.clip(signal, 0, 1)
        assert signal.shape == (8,)
        return signal


if __name__ == '__main__':
    node = Action_decoder()
    rospy.init_node('action_decoder', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down node action_decoder")
