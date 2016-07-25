#!/usr/bin/env python

import collections
import cv2
import numpy as np
import rospy

# from audio_common_msgs.msg import AudioData
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import ChannelFloat32, Image


class View():
    def __init__(self):
        self.bridge = CvBridge()
        self.buffer = collections.deque(maxlen=8192)
        self.im = np.zeros((256, 1300, 3), np.uint8)
        self.pub = rospy.Publisher("image", Image, queue_size=1)
        self.sub = rospy.Subscriber("decoded", ChannelFloat32,
                                    self.audio_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.update)

    def audio_callback(self, msg):
        for i in range(len(msg.values)):
            self.buffer.append(msg.values[i])

    def update(self, event):
        self.im /= 2  # np.zeros((700, 1000, 3), np.uint8)
        width = self.im.shape[1]
        height = self.im.shape[0]
        for i in range(width):
            if i >= len(self.buffer):
                break
            sample = self.buffer[i]
            sample *= height/2
            sample += height/2
            y = int(sample) % height
            self.im[y, i, :] = 255
        self.pub.publish(self.bridge.cv2_to_imgmsg(self.im, "bgr8"))

if __name__ == '__main__':
    rospy.init_node('view_audio')
    view = View()
    rospy.spin()
