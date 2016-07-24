#!/usr/bin/env python

import cv2
import numpy as np
import rospy

from audio_common_msgs.msg import AudioData
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class View():
    def __init__(self):
        self.bridge = CvBridge()
        self.im = np.zeros((700, 1000, 3), np.uint8)
        self.pub = rospy.Publisher("image", Image, queue_size=1)
        self.sub = rospy.Subscriber("decoded", AudioData,
                                    self.audio_callback, queue_size=1)

    def audio_callback(self, msg):
        self.im /= 2  # np.zeros((700, 1000, 3), np.uint8)
        for i in range(len(msg.data) / 2):
            if i >= self.im.shape[1]:
                break
            sa = np.uint8(ord(msg.data[i * 2][0]) + 128)
            sb = np.uint8(ord(msg.data[i * 2 + 1][0]) + 128)  # - 128
            sample = sb  # + sb * 255
            # sample /= 255
            hht = self.im.shape[0]/2
            sample += hht
            # if i > 950:
            #     print i, y
            self.im[sample, i, :] = 255
            self.im[hht, i, 1] = 255
            # self.im[i, 0, 0] = 255
        self.pub.publish(self.bridge.cv2_to_imgmsg(self.im, "bgr8"))

if __name__ == '__main__':
    rospy.init_node('view_audio')
    view = View()
    rospy.spin()
