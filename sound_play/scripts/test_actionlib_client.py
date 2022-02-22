#! /usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
import rclpy
import rclpy.action

from sound_play.action import SoundRequest as SoundRequestAction
from sound_play.msg import SoundRequest


def sound_play_client(node, volume=1.0):
    client = rclpy.action.ActionClient(
        node, SoundRequestAction, 'sound_play')
    client.wait_for_server()

    node.get_logger().info("Need Unplugging")
    goal = SoundRequestAction.Goal()
    goal.sound_request.sound = SoundRequest.NEEDS_UNPLUGGING
    goal.sound_request.command = SoundRequest.PLAY_ONCE
    goal.sound_request.volume = volume

    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    node.get_logger().info("End Need Unplugging")

    node.get_logger().info("Need Plugging")
    goal = SoundRequestAction.Goal()
    goal.sound_request.sound = SoundRequest.NEEDS_PLUGGING
    goal.sound_request.command = SoundRequest.PLAY_ONCE
    goal.sound_request.volume = volume
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    node.get_logger().info("End Need Plugging")

    node.get_logger().info("Say")
    goal = SoundRequestAction.Goal()
    goal.sound_request.sound = SoundRequest.SAY
    goal.sound_request.command = SoundRequest.PLAY_ONCE
    goal.sound_request.arg = "Testing the actionlib interface API"
    goal.sound_request.volume = volume
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    node.get_logger().info("End Say")

    node.get_logger().info("Wav")
    goal = SoundRequestAction.Goal()
    goal.sound_request.sound = SoundRequest.PLAY_FILE
    goal.sound_request.command = SoundRequest.PLAY_ONCE
    goal.sound_request.arg = os.path.join(
        get_package_share_directory('sound_play'), 'sounds') + "/say-beep.wav"
    goal.sound_request.volume = volume
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    node.get_logger().info("End wav")


if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('soundplay_client_test')
    sound_play_client(node)
