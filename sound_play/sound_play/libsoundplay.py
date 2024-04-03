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

from ament_index_python.packages import get_package_share_directory
import rclpy
import rclpy.action

from action_msgs.msg import GoalStatusArray
from sound_play_msgs.action import SoundRequest as SoundRequestAction
from sound_play_msgs.msg import SoundRequest


# \brief Class that publishes messages to the sound_play node.
#
# This class is a helper class for communicating with the sound_play node
# via the \ref sound_play.SoundRequest message. It has two ways of being used:
#
# - It can create Sound classes that represent a particular sound which
#   can be played, repeated or stopped.
#
# - It provides methods for each way in which the sound_play.SoundRequest
#   message can be invoked.

class Sound(object):
    def __init__(self, client, snd, arg, volume=1.0):
        self.client = client
        self.snd = snd
        self.arg = arg
        self.vol = volume

    # \brief Play the Sound.
    #
    # This method causes the Sound to be played once.

    def play(self, **kwargs):
        self.client.sendMsg(self.snd, SoundRequest.PLAY_ONCE, self.arg,
                            vol=self.vol, **kwargs)

    # \brief Play the Sound repeatedly.
    #
    # This method causes the Sound to be played repeatedly until stop() is
    # called.

    def repeat(self, **kwargs):
        self.client.sendMsg(
            self.snd, SoundRequest.PLAY_START, self.arg,
            vol=self.vol, **kwargs)

    # \brief Stop Sound playback.
    #
    # This method causes the Sound to stop playing.

    def stop(self):
        self.client.sendMsg(self.snd, SoundRequest.PLAY_STOP, self.arg)


# This class is a helper class for communicating with the sound_play node
# via the \ref sound_play.SoundRequest message. There is a one-to-one mapping
# between methods and invocations of the \ref sound_play.SoundRequest message.

class SoundClient(object):
    def __init__(
            self, node, blocking=False, sound_action='sound_play',
            sound_topic='robotsound'
    ):
        """Sound client for soundplay_node.

        The SoundClient can send SoundRequests in two modes: non-blocking mode
        (by publishing a message to the soundplay_node directly) which will
        return as soon as the sound request has been sent, or blocking mode (by
        using the actionlib interface) which will wait until the sound has
        finished playing completely.
        The blocking parameter here is the standard behavior, but can be
        over-ridden.  Each say/play/start/repeat method can take in an optional
        `blocking=True|False` argument that will over-ride the class-wide
        behavior. See soundclient_example.py for an example of this behavior.
        :param blocking: Used as the default behavior unless over-ridden,
        (default = false)
        :param sound_action: Namespace of actionlib to play sound.
        The actionlib interface is used
        only if blocking parameter is True. (default='sound_play')
        :param sound_topic: Topic name to play sound.
        The topic interface is used only if blocking
        parameter is False. (default='robotsound')
        """

        self._blocking = blocking
        self._playing = False

        # NOTE: only one of these will be used at once, but we need to create
        # both the publisher and actionlib client here.
        self.node = node
        self.actionclient = rclpy.action.ActionClient(
            self.node, SoundRequestAction, sound_action)
        self.pub = self.node.create_publisher(SoundRequest, sound_topic, 5)
        self.sub = self.node.create_subscription(
            GoalStatusArray,
            '{}/_action/status'.format(sound_action),
            self._action_status_cb,
            1)

    # \brief Create a voice Sound.
    #
    # Creates a Sound corresponding to saying the indicated text.
    #
    # \param s Text to say

    def voiceSound(self, s, volume=1.0):
        return Sound(self, SoundRequest.SAY, s, volume=volume)

    # \brief Create a wave Sound.
    #
    # Creates a Sound corresponding to indicated file.
    #
    # \param s File to play. Should be an absolute path that exists on the
    # machine running the sound_play node.

    def waveSound(self, sound, volume=1.0):
        if sound[0] != "/":
            rootdir = os.path.join(
                get_package_share_directory('sound_play'), 'sounds')
            sound = rootdir + "/" + sound
        return Sound(self, SoundRequest.PLAY_FILE, sound, volume=volume)

    # \brief Create a builtin Sound.
    #
    # Creates a Sound corresponding to indicated builtin wave.
    #
    # \param id Identifier of the sound to play.

    def builtinSound(self, id, volume=1.0):
        return Sound(self, id, "", volume)

    # \brief Say a string
    #
    # Send a string to be said by the sound_node. The vocalization can be
    # stopped using stopSaying or stopAll.
    #
    # \param text String to say

    def say(self, text, voice='', volume=1.0, **kwargs):
        self.sendMsg(SoundRequest.SAY, SoundRequest.PLAY_ONCE, text, voice,
                     volume, **kwargs)

    # \brief Say a string repeatedly
    #
    # The string is said repeatedly until stopSaying or stopAll is used.
    #
    # \param text String to say repeatedly

    def repeat(self, text, volume=1.0, **kwargs):
        self.sendMsg(SoundRequest.SAY, SoundRequest.PLAY_START, text,
                     vol=volume, **kwargs)

    # \brief Stop saying a string
    #
    # Stops saying a string that was previously started by say or repeat. The
    # argument indicates which string to stop saying.
    #
    # \param text Same string as in the say or repeat command

    def stopSaying(self, text):
        self.sendMsg(SoundRequest.SAY, SoundRequest.PLAY_STOP, text)

    # \brief Plays a WAV or OGG file
    #
    # Plays a WAV or OGG file once. The playback can be stopped by stopWave or
    # stopAll.
    #
    # \param sound Filename of the WAV or OGG file. Must be an absolute path
    # valid on the computer on which the sound_play node is running

    def playWave(self, sound, volume=1.0, **kwargs):
        if sound[0] != "/":
            rootdir = os.path.join(
                 get_package_share_directory('sound_play'), 'sounds')
            sound = rootdir + "/" + sound
        self.sendMsg(SoundRequest.PLAY_FILE, SoundRequest.PLAY_ONCE, sound,
                     vol=volume, **kwargs)

    # \brief Plays a WAV or OGG file repeatedly
    #
    # Plays a WAV or OGG file repeatedly until stopWave or stopAll is used.
    #
    # \param sound Filename of the WAV or OGG file. Must be an absolute path
    # valid on the computer on which the sound_play node is running.

    def startWave(self, sound, volume=1.0, **kwargs):
        if sound[0] != "/":
            rootdir = os.path.join(
                get_package_share_directory('sound_play'), 'sounds')
            sound = rootdir + "/" + sound
        self.sendMsg(SoundRequest.PLAY_FILE, SoundRequest.PLAY_START, sound,
                     vol=volume, **kwargs)

    #  \brief Stop playing a WAV or OGG file
    #
    # Stops playing a file that was previously started by playWave or
    # startWave.
    #
    # \param sound Same string as in the playWave or startWave command

    def stopWave(self, sound):
        if sound[0] != "/":
            rootdir = os.path.join(
                get_package_share_directory('sound_play'), 'sounds')
            sound = rootdir + "/" + sound
        self.sendMsg(SoundRequest.PLAY_FILE, SoundRequest.PLAY_STOP, sound)

    # \brief Plays a WAV or OGG file
    #
    # Plays a WAV or OGG file once. The playback can be stopped by
    # stopWaveFromPkg or stopAll.
    #
    # \param package Package name containing the sound file.
    # \param sound Filename of the WAV or OGG file. Must be an path relative to
    # the package valid on the computer on which the sound_play node is running

    def playWaveFromPkg(self, package, sound, volume=1.0, **kwargs):
        self.sendMsg(
            SoundRequest.PLAY_FILE,
            SoundRequest.PLAY_ONCE,
            sound, package, volume, **kwargs)

    # \brief Plays a WAV or OGG file repeatedly
    #
    # Plays a WAV or OGG file repeatedly
    # until stopWaveFromPkg or stopAll is used.
    #
    # \param package Package name containing the sound file.
    # \param sound Filename of the WAV or OGG file.
    # Must be an path relative to the package valid
    # on the computer on which the sound_play node is running

    def startWaveFromPkg(self, package, sound, volume=1.0, **kwargs):
        self.sendMsg(SoundRequest.PLAY_FILE, SoundRequest.PLAY_START, sound,
                     package, volume, **kwargs)

    #  \brief Stop playing a WAV or OGG file
    #
    # Stops playing a file that was previously started by playWaveFromPkg or
    # startWaveFromPkg.
    #
    # \param package Package name containing the sound file.
    # \param sound Filename of the WAV or OGG file.
    # Must be an path relative to the package valid
    # on the computer on which the sound_play node is running

    def stopWaveFromPkg(self, sound, package):
        self.sendMsg(
            SoundRequest.PLAY_FILE, SoundRequest.PLAY_STOP, sound, package)

    # \brief Play a buildin sound
    #
    # Starts playing one of the built-in sounds. built-ing sounds
    # are documented in \ref SoundRequest.msg.
    # Playback can be stopped by stopall.
    #
    # \param sound Identifier of the sound to play.

    def play(self, sound, volume=1.0, **kwargs):
        self.sendMsg(sound, SoundRequest.PLAY_ONCE, "", vol=volume, **kwargs)

    # \brief Play a buildin sound repeatedly
    #
    # Starts playing one of the built-in sounds repeatedly until stop or
    # stopall is used. Built-in sounds are documented in \ref SoundRequest.msg.
    #
    # \param sound Identifier of the sound to play.

    def start(self, sound, volume=1.0, **kwargs):
        self.sendMsg(sound, SoundRequest.PLAY_START, "", vol=volume, **kwargs)

    # \brief Stop playing a built-in sound
    #
    # Stops playing a built-in sound started with play or start.
    #
    # \param sound Same sound that was used to start playback

    def stop(self, sound):
        self.sendMsg(sound, SoundRequest.PLAY_STOP, "")

    # \brief Stop all currently playing sounds
    #
    # This method stops all speech, wave file, and built-in sound playback.

    def stopAll(self):
        self.stop(SoundRequest.ALL)

    def sendMsg(self, snd, cmd, s, arg2="", vol=1.0, replace=True, **kwargs):
        """Internal method for sound request.

        Internal method that publishes the sound request, either directly as a
        SoundRequest to the soundplay_node or through the actionlib interface
        (which blocks until the sound has finished playing).

        The blocking behavior is nominally the class-wide setting unless it has
        been explicitly specified in the play call.
        """

        # Use the passed-in argument if it exists, otherwise fall back to the
        # class-wide setting.
        blocking = kwargs.get('blocking', self._blocking)

        msg = SoundRequest()
        msg.sound = snd
        # Threshold volume between 0 and 1.
        msg.volume = float(max(0.0, min(1.0, vol)))
        msg.command = cmd
        msg.arg = s
        msg.arg2 = arg2

        self.node.get_logger().debug(
            'Sending sound request with volume = {}'
            ' and blocking = {}'.format(msg.volume, blocking))

        # Defensive check for the existence of the correct communicator.
        if not blocking and not self.pub:
            self.node.get_logger().error(
                'Publisher for SoundRequest must exist')
            return
        if blocking and not self.actionclient:
            self.node.get_logger().error(
                'Action client for SoundRequest does not exist.')
            return

        if not blocking:  # Publish message directly and return immediately
            self.pub.publish(msg)
            if self.pub.get_subscription_count() < 1:
                self.node.get_logger().warn(
                    "Sound command issued, but no node is subscribed"
                    " to the topic. Perhaps you forgot to run"
                    " soundplay_node.py?")
        else:  # Block until result comes back.
            assert self.actionclient, 'Actionclient must exist'
            self.node.get_logger().debug(
                'Sending action client sound request [blocking]')
            self.actionclient.wait_for_server()
            goal = SoundRequestAction.Goal()
            goal.sound_request = msg
            while not replace and self._playing:
                rclpy.sleep(0.1)
            future = self.actionclient.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, future)
            self.node.get_logger().debug(
                'sound request response received')
        return

    def _action_status_cb(self, msg):
        if any([s.status in [1, 2] for s in msg.status_list]):
            self._playing = True
        else:
            self._playing = False
