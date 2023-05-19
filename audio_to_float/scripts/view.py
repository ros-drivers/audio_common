#!/usr/bin/env python

import collections
import numpy as np
import rospy

# from audio_common_msgs.msg import AudioData
from cv_bridge import CvBridge
from sensor_msgs.msg import ChannelFloat32, Image


class View():
    def __init__(self):
        self.bridge = CvBridge()
        self.fade1 = rospy.get_param("~fade1", 0.9)
        self.fade2 = rospy.get_param("~fade2", 0.99)
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
        self.im[:, :, 1:3] = (self.im[:, :, 1:3] * self.fade1).astype(np.uint8)
        self.im[:, :, 0] = (self.im[:, :, 0] * self.fade2).astype(np.uint8)
        width = self.im.shape[1]
        height = self.im.shape[0]
        last_y = 0
        for i in range(0, width):
            if i >= len(self.buffer):
                break
            sample = self.buffer[i]
            sample *= height / 2
            sample += height / 2
            y = int(sample) % height
            y0 = min(last_y, y)
            y1 = max(last_y, y)
            self.im[y0:y1 + 1, i, :] = 255
            last_y = y
        self.pub.publish(self.bridge.cv2_to_imgmsg(self.im, "bgr8"))


if __name__ == '__main__':
    rospy.init_node('view_audio')
    view = View()
    rospy.spin()
