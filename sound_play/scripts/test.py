#!/usr/bin/env python3

# **********************************************************
#  Software License Agreement (BSD License)
#
#   Copyright (c) 2009, Willow Garage, Inc.
#   All rights reserved.
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above
#      copyright notice, this list of conditions and the following
#      disclaimer in the documentation and/or other materials provided
#      with the distribution.
#    * Neither the name of the Willow Garage nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.
# **********************************************************

import os
import time

import rclpy

from sound_play.libsoundplay import SoundClient

from sound_play.msg import SoundRequest


if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('soundplay_test' + str(os.getpid()))
    soundhandle = SoundClient(node)

    time.sleep(1)

    soundhandle.stopAll()

    node.get_logger().info(
        "This script will run continuously until you hit CTRL+C, "
        "testing various sound_node sound types.")

    node.get_logger().info('wave')
    soundhandle.playWave('say-beep.wav')
    time.sleep(2)

    node.get_logger().info('quiet wave')
    soundhandle.playWave('say-beep.wav', 0.3)
    time.sleep(2)

    node.get_logger().info('plugging')
    soundhandle.play(SoundRequest.NEEDS_PLUGGING)
    time.sleep(2)

    node.get_logger().info('quiet plugging')
    soundhandle.play(SoundRequest.NEEDS_PLUGGING, 0.3)
    time.sleep(2)

    node.get_logger().info('unplugging')
    soundhandle.play(SoundRequest.NEEDS_UNPLUGGING)
    time.sleep(2)

    node.get_logger().info('plugging badly')
    soundhandle.play(SoundRequest.NEEDS_PLUGGING_BADLY)
    time.sleep(2)

    node.get_logger().info('unplugging badly')
    soundhandle.play(SoundRequest.NEEDS_UNPLUGGING_BADLY)
    time.sleep(2)

    s1 = soundhandle.builtinSound(SoundRequest.NEEDS_UNPLUGGING_BADLY)
    s2 = soundhandle.waveSound("say-beep.wav")
    s3 = soundhandle.voiceSound("Testing the new A P I")
    s4 = soundhandle.builtinSound(SoundRequest.NEEDS_UNPLUGGING_BADLY, 0.3)
    s5 = soundhandle.waveSound("say-beep.wav", 0.3)
    s6 = soundhandle.voiceSound("Testing the new A P I", 0.3)

    node.get_logger().info("New API start voice")
    s3.repeat()
    time.sleep(3)

    node.get_logger().info("New API start voice quiet")
    s6.play()
    time.sleep(3)

    node.get_logger().info("New API wave")
    s2.repeat()
    time.sleep(2)

    node.get_logger().info("New API wave quiet")
    s5.play()
    time.sleep(2)

    node.get_logger().info("New API builtin")
    s1.play()
    time.sleep(2)

    node.get_logger().info("New API builtin quiet")
    s4.play()
    time.sleep(2)

    node.get_logger().info("New API stop")
    s3.stop()
