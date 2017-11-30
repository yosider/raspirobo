#!/usr/bin/env python
# coding: utf-8

import numpy as np
import cv2

import rospy
# TODO: use r = rospy.Rate.loop_rate(n) & r.sleep() (proccess+sleep = n sec)
from sensor_msgs.msg import CompressedImage
from raspirobo.msg import Floats
from rospy.numpy_msg import numpy_msg

from utils import compressedImage2arr
from build_models import build_visual_encoder

topic_name = "/raspicam_node/image/compressed"


class Image_encoder():
    def __init__(self):
        self.subscriber = rospy.Subscriber(topic_name, CompressedImage, self.callback, queue_size=1, buff_size=2**24)
        self.publisher = rospy.Publisher('/raspirobo/image_features', numpy_msg(Floats), queue_size=1)
        print("image_encoder initialization done!")

        self.model = build_visual_encoder()
        # initialization ??
        self.model.predict(np.zeros([1,224,224,3]))

    def callback(self, msg):
        #print("callback called!")
        img = compressedImage2arr(msg)
        img = self.preprocessor(img)
        pred = self.model.predict(img)

        # publishのshapeは(32,).
        # subscriberがexpand_dimsする.
        # numpy_msgが多次元配列に対応していない？
        feature = self.model.predict(img)[0]
        #print(feature)
        #print(type(feature))
        print "feature shape:", feature.shape
        self.publisher.publish(feature)
        #print("published!")

    def preprocessor(self, img):
        # only for image_data_format "channels_first".

        img = np.array(img, dtype=np.float32)
        #cv2.imwrite('img.jpg', img)

        # Insert a new dimension for the batch_size
        img = np.expand_dims(img, axis=0)
        assert img.shape == (1, 224, 224, 3)

        return img


if __name__ == '__main__':
    node = Image_encoder()
    rospy.init_node('image_encoder', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down node image_processor")
