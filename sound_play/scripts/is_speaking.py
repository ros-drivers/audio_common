#!/usr/bin/env python


from actionlib_msgs.msg import GoalStatusArray
import rospy
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

        rospy.Timer(rospy.Duration(0.01), self.speech_timer_cb)

    def callback(self, msg):
        if any([len(status.text) > 0 for status in msg.status_list]):
            self.is_speaking = True
        else:
            self.is_speaking = False

    def speech_timer_cb(self, timer):
        self.pub_speech_flag.publish(
            std_msgs.msg.Bool(self.is_speaking))


if __name__ == '__main__':
    rospy.init_node('is_speaking')
    IsSpeaking()
    rospy.spin()
