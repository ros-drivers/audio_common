#!/usr/bin/env python

# ***********************************************************
# * Software License Agreement (BSD License)
# *
# *  Copyright (c) 2009, Willow Garage, Inc.
# *  All rights reserved.
# *
# *  Redistribution and use in source and binary forms, with or without
# *  modification, are permitted provided that the following conditions
# *  are met:
# *
# *   * Redistributions of source code must retain the above copyright
# *     notice, this list of conditions and the following disclaimer.
# *   * Redistributions in binary form must reproduce the above
# *     copyright notice, this list of conditions and the following
# *     disclaimer in the documentation and/or other materials provided
# *     with the distribution.
# *   * Neither the name of the Willow Garage nor the names of its
# *     contributors may be used to endorse or promote products derived
# *     from this software without specific prior written permission.
# *
# *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# *  POSSIBILITY OF SUCH DAMAGE.
# ***********************************************************

import os
import sys
import threading
import traceback
import yaml

import actionlib
import roslib
import rospkg
import rospy

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from sound_play.msg import SoundRequest
from sound_play.msg import SoundRequestAction
from sound_play.msg import SoundRequestFeedback
from sound_play.msg import SoundRequestResult
from sound_play.sound_type import SoundType

try:
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import GObject as GObject
    from gi.repository import Gst as Gst
except Exception:
    str = """
**************************************************************
Error opening pygst. Is gstreamer installed?
**************************************************************
"""
    rospy.logfatal(str)
    # print str
    exit(1)


class SoundPlayNode(object):
    _feedback = SoundRequestFeedback()
    _result = SoundRequestResult()

    def stopdict(self, dict):
        for sound in dict.values():
            sound.stop()

    def stopall(self):
        self.stopdict(self.builtinsounds)
        self.stopdict(self.filesounds)
        self.stopdict(self.voicesounds)

    def select_sound(self, data):
        if data.sound == SoundRequest.PLAY_FILE:
            if not data.arg2:
                if data.arg not in self.filesounds.keys():
                    rospy.logdebug(
                        'command for uncached wave: "%s"' % data.arg)
                    try:
                        self.filesounds[data.arg] = SoundType(
                            data.arg, self.device, data.volume)
                    except Exception:
                        rospy.logerr(
                            'Error setting up to play "%s".'
                            'Does this file exist on the machine'
                            'on which sound_play is running?' % data.arg)
                        return
                else:
                    rospy.logdebug('command for cached wave: "%s"' % data.arg)
                    filesound = self.filesounds[data.arg]
                    if filesound.sound.get_property('volume') != data.volume:
                        rospy.logdebug(
                            'volume for cached wave has changed,'
                            'resetting volume')
                        filesound.sound.set_property('volume', data.volume)
                sound = self.filesounds[data.arg]
            else:
                absfilename = os.path.join(
                    roslib.packages.get_pkg_dir(data.arg2), data.arg)
                if absfilename not in self.filesounds.keys():
                    rospy.logdebug(
                        'command for uncached wave: "%s"' % absfilename)
                    try:
                        self.filesounds[absfilename] = SoundType(
                            absfilename, self.device, data.volume)
                    except Exception:
                        rospy.logerr(
                            'Error setting up to play "%s" from package "%s".'
                            'Does this file exist on the machine '
                            'on which sound_play is running?'
                            % (data.arg, data.arg2))
                        return
                else:
                    rospy.logdebug(
                        'command for cached wave: "%s"' % absfilename)
                    filesound = self.filesounds[absfilename]
                    if filesound.sound.get_property('volume') != data.volume:
                        rospy.logdebug(
                            'volume for cached wave has changed,'
                            'resetting volume')
                        filesound.sound.set_property('volume', data.volume)
                sound = self.filesounds[absfilename]
        elif data.sound == SoundRequest.SAY:
            voice_key = data.arg + '---' + data.arg2
            if voice_key not in self.voicesounds.keys():
                rospy.logdebug('command for uncached text: "%s"' % voice_key)
                if self.plugin is None:
                    rospy.logerr(
                        'Plugin is not found {}.'.format(self.plugin_name))
                else:
                    if data.arg2 == '':
                        voice = self.default_voice
                    else:
                        voice = data.arg2
                    wavfilename = self.plugin.sound_play_say_plugin(
                        data.arg, voice)
                    if wavfilename is None:
                        rospy.logerr('Failed to generate wavfile.')
                    else:
                        self.voicesounds[voice_key] = SoundType(
                            wavfilename, self.device, data.volume)
            else:
                rospy.logdebug('command for cached text: "%s"' % voice_key)
                voicesound = self.voicesounds[voice_key]
                if voicesound.sound.get_property('volume') != data.volume:
                    rospy.logdebug(
                        'volume for cached text has changed, resetting volume')
                    voicesound.sound.set_property('volume', data.volume)
            sound = self.voicesounds[voice_key]
        else:
            rospy.logdebug('command for builtin wave: %i' % data.sound)
            if ((data.sound in self.builtinsounds and
                 data.volume != self.builtinsounds[data.sound].volume)
                    or data.sound not in self.builtinsounds):
                params = self.builtinsoundparams[data.sound]
                volume = data.volume
                # use the second param as a scaling for the input volume
                if params[1] != 1:
                    volume = (volume + params[1])/2
                self.builtinsounds[data.sound] = SoundType(
                    params[0], self.device, volume)
            sound = self.builtinsounds[data.sound]
        if sound.staleness != 0 and data.command != SoundRequest.PLAY_STOP:
            # This sound isn't counted in active_sounds
            rospy.logdebug("activating %i %s" % (data.sound, data.arg))
            self.active_sounds = self.active_sounds + 1
            sound.staleness = 0
        return sound

    def callback(self, data):
        if not self.initialized:
            return
        self.mutex.acquire()
        try:
            if (data.sound == SoundRequest.ALL
                    and data.command == SoundRequest.PLAY_STOP):
                self.stopall()
            else:
                sound = self.select_sound(data)
                sound.command(data.command)
        except Exception as e:
            rospy.logerr('Exception in callback: %s' % str(e))
            rospy.loginfo(traceback.format_exc())
        finally:
            self.mutex.release()
            rospy.logdebug("done callback")

    # Purge sounds that haven't been played in a while.
    def cleanupdict(self, dict):
        purgelist = []
        for key, sound in iter(dict.items()):
            try:
                staleness = sound.get_staleness()
            except Exception as e:
                rospy.logerr(
                    'Exception in cleanupdict for sound (%s): %s'
                    % (str(key), str(e)))
                # Something is wrong. Let's purge and try again.
                staleness = 100
            # print "%s %i"%(key, staleness)
            if staleness >= 10:
                purgelist.append(key)
            # Sound is playing
            if staleness == 0:
                self.active_sounds = self.active_sounds + 1
        for key in purgelist:
            rospy.logdebug('Purging %s from cache' % key)
            # clean up resources
            dict[key].dispose()
            del dict[key]

    def cleanup(self):
        self.mutex.acquire()
        try:
            self.active_sounds = 0
            self.cleanupdict(self.filesounds)
            self.cleanupdict(self.voicesounds)
            self.cleanupdict(self.builtinsounds)
        except Exception:
            rospy.loginfo(
                'Exception in cleanup: %s' % sys.exc_info()[0])
        finally:
            self.mutex.release()

    def diagnostics(self, state):
        try:
            da = DiagnosticArray()
            ds = DiagnosticStatus()
            ds.name = rospy.get_caller_id().lstrip('/') + ": Node State"
            if state == 0:
                ds.level = DiagnosticStatus.OK
                ds.message = "%i sounds playing" % self.active_sounds
                ds.values.append(
                    KeyValue("Active sounds", str(self.active_sounds)))
                ds.values.append(
                    KeyValue(
                        "Allocated sound channels",
                        str(self.num_channels)))
                ds.values.append(
                    KeyValue(
                        "Buffered builtin sounds",
                        str(len(self.builtinsounds))))
                ds.values.append(
                    KeyValue(
                        "Buffered wave sounds",
                        str(len(self.filesounds))))
                ds.values.append(
                    KeyValue(
                        "Buffered voice sounds",
                        str(len(self.voicesounds))))
            elif state == 1:
                ds.level = DiagnosticStatus.WARN
                ds.message = "Sound device not open yet."
            else:
                ds.level = DiagnosticStatus.ERROR
                ds.message = "Can't open sound device." +\
                    "See http://wiki.ros.org/sound_play/Troubleshooting"
            da.status.append(ds)
            da.header.stamp = rospy.get_rostime()
            self.diagnostic_pub.publish(da)
        except Exception as e:
            rospy.loginfo('Exception in diagnostics: %s' % str(e))

    def execute_cb(self, data):
        data = data.sound_request
        if not self.initialized:
            rospy.logerr('soundplay_node is not initialized yet.')
            self._as.set_aborted()
            return
        self.mutex.acquire()
        # Force only one sound at a time
        self.stopall()
        try:
            if (data.sound == SoundRequest.ALL
                    and data.command == SoundRequest.PLAY_STOP):
                self.stopall()
            else:
                sound = self.select_sound(data)
                sound.command(data.command)

                r = rospy.Rate(self.loop_rate)
                start_time = rospy.get_rostime()
                success = True
                while sound.get_playing():
                    sound.update()
                    if self._as.is_preempt_requested():
                        rospy.loginfo('sound_play action: Preempted')
                        sound.stop()
                        self._as.set_preempted()
                        success = False
                        break

                    self._feedback.playing = sound.get_playing()
                    self._feedback.stamp = rospy.get_rostime() - start_time
                    self._as.publish_feedback(self._feedback)
                    r.sleep()

                if success:
                    self._result.playing = self._feedback.playing
                    self._result.stamp = self._feedback.stamp
                    rospy.loginfo('sound_play action: Succeeded')
                    self._as.set_succeeded(self._result)

        except Exception as e:
            self._as.set_aborted()
            rospy.logerr(
                'Exception in actionlib callback: %s' % str(e))
            rospy.loginfo(traceback.format_exc())
        finally:
            self.mutex.release()
            rospy.logdebug("done actionlib callback")

    def __init__(self):
        Gst.init(None)

        # Start gobject thread to receive gstreamer messages
        GObject.threads_init()
        self.g_loop = threading.Thread(target=GObject.MainLoop().run)
        self.g_loop.daemon = True
        self.g_loop.start()

        rospy.init_node('sound_play')
        self.loop_rate = rospy.get_param('~loop_rate', 100)
        self.device = rospy.get_param("~device", "default")
        self.default_voice = rospy.get_param('~default_voice', None)
        self.plugin_name = rospy.get_param(
            '~plugin', 'sound_play/festival_plugin')
        self.diagnostic_pub = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=1)
        rootdir = os.path.join(
            roslib.packages.get_pkg_dir('sound_play'), 'sounds')

        # load plugin
        rospack = rospkg.RosPack()
        depend_pkgs = rospack.get_depends_on('sound_play', implicit=False)
        depend_pkgs = ['sound_play'] + depend_pkgs
        rospy.loginfo("Loading from plugin definitions")
        plugin_yamls = []
        for depend_pkg in depend_pkgs:
            manifest = rospack.get_manifest(depend_pkg)
            plugin_yaml = manifest.get_export('sound_play', 'plugin')
            if len(plugin_yaml) != 0:
                plugin_yamls += plugin_yaml
                for plugin_y in plugin_yaml:
                    rospy.logdebug('Loading plugin in {}'.format(plugin_y))
        plugin_dict = {}
        for plugin_yaml in plugin_yamls:
            if not os.path.exists(plugin_yaml):
                rospy.logerr(
                    'Failed to load plugin yaml: {}'.format(plugin_yaml))
                rospy.logerr(
                    'Missing plugin yaml: {}'.format(plugin_yaml))
                continue
            with open(plugin_yaml) as f:
                plugin_descs = yaml.safe_load(f)
            for plugin_desc in plugin_descs:
                plugin_dict[plugin_desc['name']] = plugin_desc['module']

        self.plugin = None
        if self.plugin_name in plugin_dict.keys():
            plugin_module = plugin_dict[self.plugin_name]
            mod = __import__(plugin_module.split('.')[0])
            for sub_mod in plugin_module.split('.')[1:]:
                mod = getattr(mod, sub_mod)
            self.plugin = mod()

        self.builtinsoundparams = {
            SoundRequest.BACKINGUP: (
                os.path.join(rootdir, 'BACKINGUP.ogg'), 0.1),
            SoundRequest.NEEDS_UNPLUGGING: (
                os.path.join(rootdir, 'NEEDS_UNPLUGGING.ogg'), 1),
            SoundRequest.NEEDS_PLUGGING: (
                os.path.join(rootdir, 'NEEDS_PLUGGING.ogg'), 1),
            SoundRequest.NEEDS_UNPLUGGING_BADLY: (
                os.path.join(rootdir, 'NEEDS_UNPLUGGING_BADLY.ogg'), 1),
            SoundRequest.NEEDS_PLUGGING_BADLY: (
                os.path.join(rootdir, 'NEEDS_PLUGGING_BADLY.ogg'), 1),
        }

        self.no_error = True
        self.initialized = False
        self.active_sounds = 0

        self.mutex = threading.Lock()
        self.sub = rospy.Subscriber("robotsound", SoundRequest, self.callback)
        self._as = actionlib.SimpleActionServer(
            'sound_play', SoundRequestAction,
            execute_cb=self.execute_cb, auto_start=False)

        self.mutex.acquire()
        # For ros startup race condition
        self.sleep(0.5)
        self.diagnostics(1)

        while not rospy.is_shutdown():
            while not rospy.is_shutdown():
                self.init_vars()
                self.no_error = True
                self.initialized = True
                self.mutex.release()
                if not self._as.action_server.started:
                    self._as.start()
                try:
                    self.idle_loop()
                    # Returns after inactive period to test device availability
                    # print "Exiting idle"
                except Exception:
                    rospy.loginfo(
                        'Exception in idle_loop: %s' % sys.exc_info()[0])
                finally:
                    self.mutex.acquire()

            self.diagnostics(2)
        self.mutex.release()

    def init_vars(self):
        self.num_channels = 10
        self.builtinsounds = {}
        self.filesounds = {}
        self.voicesounds = {}
        self.hotlist = []
        if not self.initialized:
            rospy.loginfo('sound_play node is ready to play sound')

    def sleep(self, duration):
        try:
            rospy.sleep(duration)
        except rospy.exceptions.ROSInterruptException:
            pass

    def get_sound_length(self):
        sound_length = len(self.builtinsounds) +\
            len(self.voicesounds) + len(self.filesounds)
        return sound_length

    def idle_loop(self):
        self.last_activity_time = rospy.get_time()
        while (not rospy.is_shutdown()
                and (rospy.get_time() - self.last_activity_time < 10
                     or self.get_sound_length() > 0)):
            # print("idle_loop")
            self.diagnostics(0)
            self.sleep(1)
            self.cleanup()
        # print("idle_exiting")


if __name__ == '__main__':
    SoundPlayNode()
