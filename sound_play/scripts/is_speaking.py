#!/usr/bin/env python3

import rclpy
import rclpy.node

from action_msgs.msg import GoalStatus
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
        self.timer = self.create_timer(0.01, self.speech_timer_cb)

    def __del__(self):
        self.destroy_timer(self.timer)

    def check_speak_status(self, status_msg):
        """Returns True when speaking.

        """
        # If it is not included in the terminal state,
        # it is determined as a speaking state.
        if status_msg.status in [GoalStatus.STATUS_ACCEPTED,
                                 GoalStatus.STATUS_EXECUTING]:
            return True
        return False

    def callback(self, msg):
        for status in msg.status_list:
            if self.check_speak_status(status):
                self.is_speaking = True
                return
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
