# coding: utf-8

import numpy as np
import cv2

def compressedImage2arr(msg):
    data = msg.data
    arr = np.fromstring(data, np.uint8)
    arr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    return arr
