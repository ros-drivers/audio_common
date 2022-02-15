#!/usr/bin/env python3

"""
Simple example showing how to use the SoundClient provided by libsoundplay,
in blocking, non-blocking, and explicit usage.
"""
import os
import time

import rclpy
import rclpy.utilities

from sound_play.libsoundplay import SoundClient

from sound_play.msg import SoundRequest


def play_explicit(node):
    node.get_logger().info(
        'Example: SoundClient play methods can take in an explicit'
        ' blocking parameter')
    # blocking = False by default
    soundhandle = SoundClient(node, blocking=False)
    time.sleep(0.5)  # Ensure publisher connection is successful.

    sound_beep = soundhandle.waveSound("say-beep.wav", volume=0.5)
    # Play the same sound twice, once blocking and once not. The first call is
    # blocking (explicitly specified).
    sound_beep.play(blocking=True)
    # This call is not blocking (uses the SoundClient's setting).
    sound_beep.play()
    time.sleep(0.5)  # Let sound complete.

    # Play a blocking sound.
    soundhandle.play(SoundRequest.NEEDS_UNPLUGGING, blocking=True)

    # Create a new SoundClient where the default behavior *is* to block.
    soundhandle = SoundClient(node, blocking=True)
    soundhandle.say('Say-ing stuff while block-ing')
    soundhandle.say('Say-ing stuff without block-ing', blocking=False)
    time.sleep(1)


def play_blocking(node):
    """Play various sounds.

    Play various sounds, blocking
    until each is completed before going to the next.
    """
    node.get_logger().info('Example: Playing sounds in *blocking* mode.')
    soundhandle = SoundClient(node, blocking=True)

    node.get_logger().info('Playing say-beep at full volume.')
    soundhandle.playWave('say-beep.wav')

    node.get_logger().info('Playing say-beep at volume 0.3.')
    soundhandle.playWave('say-beep.wav', volume=0.3)

    node.get_logger().info('Playing sound for NEEDS_PLUGGING.')
    soundhandle.play(SoundRequest.NEEDS_PLUGGING)

    node.get_logger().info('Speaking some long string.')
    soundhandle.say('It was the best of times, it was the worst of times.')


def play_nonblocking(node):
    "Play the same sounds with manual pauses between them."

    node.get_logger().info('Example: Playing sounds in *non-blocking* mode.')
    # NOTE: you must sleep at the beginning to let the SoundClient publisher
    # establish a connection to the soundplay_node.
    soundhandle = SoundClient(node, blocking=False)
    time.sleep(1)

    # In the non-blocking version you need to sleep between calls.
    node.get_logger().info('Playing say-beep at full volume.')
    soundhandle.playWave('say-beep.wav')
    time.sleep(1)

    node.get_logger().info('Playing say-beep at volume 0.3.')
    soundhandle.playWave('say-beep.wav', volume=0.3)
    time.sleep(1)

    node.get_logger().info('Playing sound for NEEDS_PLUGGING.')
    soundhandle.play(SoundRequest.NEEDS_PLUGGING)
    time.sleep(1)

    node.get_logger().info('Speaking some long string.')
    soundhandle.say('It was the best of times, it was the worst of times.')
    # Note we will return before the string has finished playing.


if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('soundclient_example' + str(os.getpid()))
    play_explicit(node)
    play_blocking(node)
    play_nonblocking(node)
    node.get_logger().info('Finished')
