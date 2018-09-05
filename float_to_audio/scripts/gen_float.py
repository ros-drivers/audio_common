#!/usr/bin/env python
# Lucas Walter
# Generate a series of float arrays where number of samples * array rate = sample rate 
# (e.g 1000 samples/msg * 16 msgs/sec = 16 KHz)
#
# TODO(lucasw)
# Make this node generate a constant stream, but default to zero
# then have it subscribe to an input topic that accepts arbitrary length
# float arrays, it will mix these into the other stream as it receives them
# receiving a high rate of incoming messages will cause them all to be summed
# together.
# In order to support this a fifo buffer will store all the samples to be
# played back, and grow it as needed to support the longest sample received.

import random
import rospy

from sensor_msgs.msg import ChannelFloat32


class GenFloat():
    def __init__(self):
        self.msg_rate = rospy.get_param("~msg_rate", 16)
        self.sample_rate = rospy.get_param("~sample_rate", 16000)
        self.samples_per_msg = int(self.sample_rate / self.msg_rate)
        # not quite 1.0 / self.msg_rate
        self.dt = float(self.samples_per_msg) / float(self.sample_rate)
        rospy.loginfo(self.dt)

        self.msg = ChannelFloat32()
        for i in range(self.samples_per_msg):
            self.msg.values.append(0)
        self.counter = int(0)

        # signal period for each 'voice'/channel
        self.period = [200, 417, 400]
        # in samples
        self.phase = [0, 0, 0]
        self.pub = rospy.Publisher("samples", ChannelFloat32, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.update)

    def update(self, event):
        for i in range(self.samples_per_msg):
            val = 0
            # square wave
            sc = 0.2
            ind = 0
            if self.counter % self.period[ind] < self.period[ind]/2:
                val += sc
            else:
                val -= sc
            # triangle
            sc = 0.3
            ind = 1
            fr = float(self.counter % self.period[ind]) / float(self.period[ind])
            if fr < 0.5:
                val += sc * (fr * 4.0 - 1.0)
            else:
                val += sc * (1.0 - ((fr - 0.5) * 4.0))
            val += random.random() * 0.2
            self.msg.values[i] = val
            self.counter += 1

        self.pub.publish(self.msg)

if __name__ == '__main__':
    rospy.init_node('gen_float')
    gen_float = GenFloat()
    rospy.spin()
