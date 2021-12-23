#!/usr/bin/env python


from actionlib_msgs.msg import GoalStatusArray
import rospy
import std_msgs.msg


class IsSpeeching(object):

    def __init__(self):
        super(IsSpeeching, self).__init__()

        self.sub = rospy.Subscriber(
            '~robotsound',
            GoalStatusArray,
            callback=self.callback,
            queue_size=1)

        self.is_speeching = False
        self.pub_speech_flag = rospy.Publisher(
            '~output/is_speeching',
            std_msgs.msg.Bool, queue_size=1)

        rospy.Timer(rospy.Duration(0.01), self.speech_timer_cb)

    def callback(self, msg):
        if any([len(status.text) > 0 for status in msg.status_list]):
            self.is_speeching = True
        else:
            self.is_speeching = False

    def speech_timer_cb(self, timer):
        self.pub_speech_flag.publish(
            std_msgs.msg.Bool(self.is_speeching))


if __name__ == '__main__':
    rospy.init_node('is_speeching')
    IsSpeeching()
    rospy.spin()
