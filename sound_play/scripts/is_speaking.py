#!/usr/bin/env python

import rospy

from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatusArray
import std_msgs.msg


class IsSpeaking(object):

    def __init__(self):
        super(IsSpeaking, self).__init__()

        self.sub = rospy.Subscriber(
            '~robotsound',
            GoalStatusArray,
            callback=self.callback,
            queue_size=1)

        self.is_speaking = False
        self.pub_speech_flag = rospy.Publisher(
            '~output/is_speaking',
            std_msgs.msg.Bool, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.01), self.speech_timer_cb)

    def check_speak_status(self, status_msg):
        """Returns True when speaking.

        """
        # If it is not included in the terminal state,
        # it is determined as a speaking state.
        if status_msg.status in [GoalStatus.ACTIVE,
                                 GoalStatus.PREEMPTING,
                                 GoalStatus.RECALLING]:
            return True
        return False

    def callback(self, msg):
        for status in msg.status_list:
            if self.check_speak_status(status):
                self.is_speaking = True
                return
        self.is_speaking = False

    def speech_timer_cb(self, timer):
        self.pub_speech_flag.publish(
            std_msgs.msg.Bool(self.is_speaking))


if __name__ == '__main__':
    rospy.init_node('is_speaking')
    app = IsSpeaking()
    rospy.spin()
