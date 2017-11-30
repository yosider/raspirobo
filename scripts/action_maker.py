#!/usr/bin/env python
# coding: utf-8

import numpy as np

import rospy
from raspirobo.msg import Floats
from rospy.numpy_msg import numpy_msg

from build_models import build_action_maker

class Action_maker():
    ''' Subscribe features and Publish abstract action signal '''
    def __init__(self):
        self.subscriber = rospy.Subscriber("/raspirobo/image_features", numpy_msg(Floats), self.callback, queue_size=1, buff_size=2**24)
        self.publisher = rospy.Publisher("/raspirobo/action", numpy_msg(Floats), queue_size=1)
        self.model = build_action_maker()
        self.model.predict(np.zeros([1,32]))

    def callback(self, msg):
        #print("callback called!")
        state = self.msg2state(msg)
        #print 'msg:', msg
        #print 'data', msg.data
        action = self.get_action(state)  # ndarray
        print "action:", action
        self.publisher.publish(action)
        #print("action published!")

    def msg2state(self, msg):
        assert msg.data.shape == (32,)
        # 多次元配列にもどす(batch_sizeの次元追加)
        # numpy_msgが多次元配列に対応していないため
        state = np.expand_dims(msg.data, axis=0)
        return state

    def get_action(self, state):
        # output action element is model output, in [0., 1.]
        action = self.model.predict(state)[0]
        action = np.clip(action, 0, 1)
        assert action.shape == (16,)
        return action


if __name__ == '__main__':
    node = Action_maker()
    rospy.init_node('action_maker', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down node action_maker")
