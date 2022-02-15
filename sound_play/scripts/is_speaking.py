#!/usr/bin/env python3

import rclpy
import rclpy.node

from action_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool


class IsSpeakingNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('is_speaking')

        self.sub = self.create_subscription(
            GoalStatusArray, '~/robotsound', self.callback, 1)

        self.is_speaking = False
        self.pub_speech_flag = self.create_publisher(
            Bool, '~/output/is_speaking', 1)
        self.create_timer(0.01, self.speech_timer_cb)

    def __del__(self):
        self.destroy_timer(self.timer)
        self.dispose()

    def callback(self, msg):
        if any([s.status in [1, 2] for s in msg.status_list]):
            self.is_speaking = True
        else:
            self.is_speaking = False

    def speech_timer_cb(self):
        self.pub_speech_flag.publish(
            Bool(data=self.is_speaking))


if __name__ == '__main__':
    rclpy.init()
    is_speaking_node = IsSpeakingNode()
    rclpy.spin(is_speaking_node)
    is_speaking_node.destroy_node()
    del is_speaking_node
    rclpy.shutdown()
