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
        self.im = np.zeros((700, 1000, 1), np.uint8)
        self.pub = rospy.Publisher("image", Image, queue_size=1)
        self.sub = rospy.Subscriber("decoded", AudioData,
                                    self.audio_callback, queue_size=1)

    def audio_callback(self, msg):
        self.im = np.zeros((700, 1000, 1), np.uint8)
        # print ord(msg.data[0][0])
        for i in range(len(msg.data)):
            if i >= self.im.shape[1]:
                print i
                break
            y = ord(msg.data[i][0])
            # if i > 950:
            #     print i, y
            self.im[y, i, 0] = 255
            # self.im[i, 0, 0] = 255
        self.pub.publish(self.bridge.cv2_to_imgmsg(self.im, "mono8"))

if __name__ == '__main__':
    rospy.init_node('view_audio')
    view = View()
    rospy.spin()
