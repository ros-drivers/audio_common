#!/usr/bin/env python

import collections
import cv2
import numpy as np
import rospy

# from audio_common_msgs.msg import AudioData
from cv_bridge import CvBridge, CvBridgeError
from scipy import signal
from sensor_msgs.msg import ChannelFloat32, Image


class View():
    def __init__(self):
        self.bridge = CvBridge()
        self.buffer_len = rospy.get_param("~buffer_len", 2**16)
        self.buffer = collections.deque(maxlen=self.buffer_len)
        self.sample_rate = rospy.get_param("~sample_rate", 44100)
        # self.window = 256
        self.im = None
        self.pub = rospy.Publisher("image_spectrogram", Image, queue_size=1)
        self.sub = rospy.Subscriber("samples", ChannelFloat32,
                                    self.audio_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.2), self.update)

    def audio_callback(self, msg):
        for i in range(len(msg.values)):
            self.buffer.append(msg.values[i])

    def update(self, event):
        if len(self.buffer) < self.buffer_len:
            return
        samples = np.asarray(self.buffer)
        f, t, Sxx = signal.spectrogram(samples, self.sample_rate)
        # TODO(lucasw) is there a standard spectrogram conversion?
        Sxx = np.log(1.0 + Sxx * 2**16)
        mins = np.min(Sxx)
        maxs = np.max(Sxx)
        Sxx -= mins
        print Sxx.shape, mins, maxs
        self.im = (Sxx * 50).astype(np.uint8)
        # self.im[y0:y1+1, i, :] = 255
        self.pub.publish(self.bridge.cv2_to_imgmsg(self.im, "mono8"))
        # rospy.signal_shutdown("")

if __name__ == '__main__':
    rospy.init_node('spectrogram')
    view = View()
    rospy.spin()
