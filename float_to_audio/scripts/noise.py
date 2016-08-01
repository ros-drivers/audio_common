#!/usr/bin/env python
# Lucas Walter
# NES style pseudo random 1-bit noise
# http://wiki.nesdev.com/w/index.php/APU_Noise

import rospy
from sensor_msgs.msg import ChannelFloat32
from std_msgs.msg import Float32


class Noise():
    def __init__(self):
        self.shift_register = 0x0001
        self.msg_rate = rospy.get_param("~msg_rate", 8)
        self.sample_rate = rospy.get_param("~sample_rate", 16000)
        self.samples_per_msg = int(self.sample_rate / self.msg_rate)
        self.dt = float(self.samples_per_msg) / float(self.sample_rate)
        self.msg = ChannelFloat32()
        for i in range(self.samples_per_msg):
            self.msg.values.append(0)
        self.counter = int(0)
        # frequency range should be from 29.3 Hz to 447 KHz
        self.frequency = 500
        self.frequency_sub = rospy.Subscriber("frequency", Float32,
                                              self.frequency_callback, queue_size=1)
        self.pub = rospy.Publisher("samples", ChannelFloat32, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.update)

    def frequency_callback(self, msg):
        self.frequency = msg.data

    def update(self, event):
        period = int(self.sample_rate / self.frequency)

        sc = 0.4
        loop = True
        cycles = 0
        while loop:
            feedback = (self.shift_register & 0x1 > 0) != (self.shift_register & (1 << 6) > 0)
            # feedback = (self.shift_register & 0x1 > 0) != (self.shift_register & 0x2 > 0)
            print self.counter, cycles, self.shift_register & 0x1, format(self.shift_register, "015b")
            self.shift_register = self.shift_register >> 1
            self.shift_register |= feedback << 14
            for j in range(self.counter % period, period):
                ind = cycles * period + j
                if ind >= self.samples_per_msg:
                    loop = False
                    break
                if not (self.shift_register & 0x1):
                    self.msg.values[ind] = sc
                else:
                    self.msg.values[ind] = 0
                self.counter += 1
            cycles += 1
            # print ("%s" % (format(self.shift_register, "04x")))
        self.pub.publish(self.msg)
        rospy.signal_shutdown("debug quit")

if __name__ == '__main__':
    rospy.init_node('noise')
    noise = Noise()
    rospy.spin()
