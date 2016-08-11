#!/usr/bin/env python
# -*- coding: utf-8 -*-
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  Copyright (c) 2016 JSK Lab.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Willow Garage nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

# Author: Blaise Gassend
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import actionlib
import os
from roslib.packages import get_pkg_dir
import rospy
import subprocess
import stat
import sys
import tempfile
from threading import Lock, RLock
import traceback
from diagnostic_msgs.msg import (DiagnosticArray,
                                 DiagnosticStatus,
                                 KeyValue)
from sound_play.msg import (SoundRequest,
                            SoundRequestAction,
                            SoundRequestResult,
                            SoundRequestFeedback)

try:
    import pygst
    pygst.require('0.10')
    import gst
    import gobject
except:
    err_string="""
**************************************************************
Error opening pygst. Is gstreamer installed? (sudo apt-get install python-gst0.10
**************************************************************
"""
    rospy.logfatal(err_string)
    print err_string
    exit(1)

class SoundState(object):
    STOPPED = 0
    LOOPING = 1
    COUNTING = 2

class SoundType(object):
    BuiltinSound = 3
    FileSound = 4
    VoiceSound = 5

class SoundObject(SoundState, SoundType):
    def __init__(self, sound_type, uri, volume=1.0):
        self.lock = RLock()
        self.sound_type = sound_type
        self.uri = uri
        self.volume = volume
        self.state = SoundState.STOPPED
        self.staleness = 1

        self.sound = gst.element_factory_make('playbin', 'player')
        self.sound.set_property('uri', self.uri)
        self.sound.set_property('volume', self.volume)

        self.bus = self.sound.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect('message', self.on_stream_end)
    def __del__(self):
        # stop our GST object so that it gets garbage-collected
        self.stop()
    def on_stream_end(self, bus, message):
        if message.type == gst.MESSAGE_EOS:
            self.state = SoundState.STOPPED
    def poll(self):
        self.bus.poll(gst.MESSAGE_ERROR, 10)
    def play(self, once=True):
        if once:
            self.single()
        else:
            self.loop()
    def loop(self):
        self.lock.acquire()
        try:
            self.staleness = 0
            if self.state == SoundState.COUNTING:
                self.stop()

            if self.state == SoundState.STOPPED:
              self.sound.seek_simple(gst.FORMAT_TIME, gst.SEEK_FLAG_FLUSH, 0)
              self.sound.set_state(gst.STATE_PLAYING)
            self.state = SoundState.LOOPING
        finally:
            self.lock.release()

    def stop(self):
        if self.state != SoundState.STOPPED:
            self.lock.acquire()
            try:
                self.sound.set_state(gst.STATE_NULL)
                self.state = SoundState.STOPPED
            finally:
                self.lock.release()

    def single(self):
        self.lock.acquire()
        try:
            rospy.logdebug("Playing %s"%self.uri)
            self.staleness = 0
            if self.state == SoundState.LOOPING:
                self.stop()

            self.sound.seek_simple(gst.FORMAT_TIME, gst.SEEK_FLAG_FLUSH, 0)
            self.sound.set_state(gst.STATE_PLAYING)
            self.state = SoundState.COUNTING
        finally:
            self.lock.release()

    def get_staleness(self):
        self.lock.acquire()
        position = 0
        duration = 0
        try:
            position = self.sound.query_position(gst.FORMAT_TIME)[0]
            duration = self.sound.query_duration(gst.FORMAT_TIME)[0]
        except Exception, e:
            position = 0
            duration = 0
        finally:
            self.lock.release()

        if position != duration:
            self.staleness = 0
        else:
            self.staleness = self.staleness + 1
        return self.staleness

    def is_playing(self):
        return self.state == SoundState.COUNTING

class SoundLoader(object):
    sounds_dir = os.path.join(get_pkg_dir('sound_play'), 'sounds')
    builtin_sound_params = {
        SoundRequest.BACKINGUP              : (os.path.join(sounds_dir, 'BACKINGUP.ogg'), 0.1),
        SoundRequest.NEEDS_UNPLUGGING       : (os.path.join(sounds_dir, 'NEEDS_UNPLUGGING.ogg'), 1),
        SoundRequest.NEEDS_PLUGGING         : (os.path.join(sounds_dir, 'NEEDS_PLUGGING.ogg'), 1),
        SoundRequest.NEEDS_UNPLUGGING_BADLY : (os.path.join(sounds_dir, 'NEEDS_UNPLUGGING_BADLY.ogg'), 1),
        SoundRequest.NEEDS_PLUGGING_BADLY   : (os.path.join(sounds_dir, 'NEEDS_PLUGGING_BADLY.ogg'), 1),
    }

    def __init__(self):
        self.sounds = {}
    def stop_all(self):
        for sound in self.sounds.values():
            sound.stop()
        rospy.logdebug("stopped all sounds")
    def clear_cache(self):
        for key, sound in self.sounds.items():
            try:
                staleness = sound.get_staleness()
                if staleness >= 10:
                    sound.stop()
                    del self.sounds[key]
            except Exception as e:
                rospy.logerr('Failed to clear cache: %s' % key)
    def uri_from_path(self, path):
        if ':' in path:
            return path
        elif os.path.isfile(path):
            return 'file://' + os.path.abspath(path)
        else:
            raise Exception('Invalid URI: %s' % path)
    def load_from_file_path(self, path, volume=1.0, sound_type=SoundType.FileSound):
        uri = self.uri_from_path(path)
        if not uri in self.sounds:
            self.sounds[uri] = SoundObject(sound_type=sound_type,
                                           uri=uri, volume=volume)
        return self.sounds[uri]
    def load_from_builtin_sound(self, sound):
        if not sound in self.builtin_sound_params:
            raise Exception('sound type %d is not in builtin_sound_prams' % sound)
        path, volume = self.builtin_sound_params[sound]
        return self.load_from_file_path(path, volume, sound_type=SoundType.BuiltinSound)
    def load_from_ros_path(self, pkg, rel_path, volume=1.0):
        file_path = os.path.join(get_pkg_dir(pkg), rel_path)
        return self.load_from_file_path(file_path, volume)
    def load_from_speech_message(self, txt, voice, volume=1.0):
        if not txt in self.sounds:
            with tempfile.NamedTemporaryFile(prefix='sound_play', suffix='.wav') as f:
                target_wav_path = f.name
            with tempfile.NamedTemporaryFile(prefix='sound_play', suffix='.txt') as txt_file:
                txt_file.write(txt)
                txt_file.flush()
                cmd = ['text2wave', '-eval', '(' + voice + ')',
                       txt_file.name, '-o', target_wav_path]
                ret_code = subprocess.check_call(cmd)
                rospy.logdebug('created wav file with command %s (return code: %d)' % (cmd, ret_code))
                if ret_code != 0 or os.stat(target_wav_path).st_size == 0:
                    raise Exception("""Sound synthesis failed.
Is festival installed? Is a festival voice installed?
Try running 'rosdep satisfy sound_play | sh'. Refer to http://wiki.ros.org/sound_play/Troubleshooting.
Executed Command: {cmd}
Return Code: {ret_code}""".format(cmd=cmd, ret_code=ret_code))
            uri = self.uri_from_path(target_wav_path)
            self.sounds[txt] = SoundObject(sound_type=SoundType.VoiceSound, uri=uri, volume=volume)
        return self.sounds[txt]
    def active_sounds(self):
        active_sounds_num = 0
        for sound in self.sounds.values():
            if sound.is_playing():
                active_sounds_num += 1
        return active_sounds_num
    def buffered_sounds(self, sound_type=None):
        return len([s for s in self.sounds.values() if s.sound_type == sound_type])

class SoundPlay(object):
    def __init__(self):
        self.lock = Lock()
        self.loader = SoundLoader()
        self.diagnostic_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)
        self.sound_request_sub = rospy.Subscriber('robotsound', SoundRequest, self.on_sound_request)
        self.action_server = actionlib.SimpleActionServer('sound_play', SoundRequestAction,
                                                          execute_cb=self.on_sound_request_action_goal,
                                                          auto_start=False)
        self.action_server.start()
        self.poll_timer = rospy.Timer(rospy.Duration(1.0), self.poll)
        self.publish_diagnostics(DiagnosticStatus.WARN)
    def poll(self, event=None):
        self.loader.clear_cache()
        self.publish_diagnostics(DiagnosticStatus.OK)
    def on_sound_request(self, msg):
        with self.lock:
            try:
                sound = self.command_from_msg(msg)
            except Exception as e:
                rospy.logerr('Failed to command request: %s' % str(e))
                rospy.logerr(traceback.format_exc())
    def on_sound_request_action_goal(self, goal):
        msg = goal.sound_request
        with self.lock:
            try:
                self.loader.stop_all()
                start_time = rospy.Time.now()
                sound = self.command_from_msg(msg)
                feedback_msg = SoundRequestFeedback()
                r = rospy.Rate(1)
                while sound.is_playing():
                    sound.poll()
                    if self.action_server.is_preempt_requested():
                        rospy.logdebug('sound_play action: Preempted')
                        sound.stop()
                        self.action_server.set_preempted()
                        return
                    feedback_msg.playing = sound.is_playing()
                    feedback_msg.stamp = rospy.Time.now() - start_time
                    self.action_server.publish_feedback(feedback_msg)
                    r.sleep()
                result_msg = SoundRequestResult()
                result_msg.playing = feedback_msg.playing
                result_msg.stamp = feedback_msg.stamp
                self.action_server.set_succeeded(result_msg)
                rospy.logdebug('sound_play action: succeeded')
            except Exception as e:
                rospy.logerr('Exception in sound_play action callback: %s' % str(e))
                rospy.logerr(traceback.format_exc())
    def clear_cache(self):
        self.loader.clear_cache()
    def command_from_msg(self, msg):
        if msg.sound == SoundRequest.ALL and msg.command == SoundRequest.PLAY_STOP:
            self.loader.stop_all()
            return None
        elif msg.sound == SoundRequest.PLAY_FILE:
            if msg.arg2 is not None and len(msg.arg2) != 0:
                sound = self.loader.load_from_ros_path(msg.arg2, msg.arg)
            else:
                sound = self.loader.load_from_file_path(msg.arg)
        elif msg.sound == SoundRequest.SAY:
            sound = self.loader.load_from_speech_message(msg.arg, msg.arg2)
        else:
            sound = self.loader.load_from_builtin_sound(msg.sound)

        if msg.command == SoundRequest.PLAY_STOP:
            sound.stop()
        elif msg.command == SoundRequest.PLAY_ONCE:
            sound.play(once=True)
        elif msg.command == SoundRequest.PLAY_START:
            sound.play(once=False)
        return sound
    def publish_diagnostics(self, level):
        try:
            status = DiagnosticStatus()
            status.name = rospy.get_caller_id().lstrip('/') + ": Node State"
            status.level = level
            if level == DiagnosticStatus.OK:
                active_sounds = self.loader.active_sounds()
                status.message = "%i sounds playing" % active_sounds
                status.values += [
                    KeyValue("Active sounds", str(active_sounds)),
                    KeyValue("Buffered builtin sounds", str(self.loader.buffered_sounds(SoundType.BuiltinSound))),
                    KeyValue("Buffered wave sounds", str(self.loader.buffered_sounds(SoundType.FileSound))),
                    KeyValue("Buffered voice sounds", str(self.loader.buffered_sounds(SoundType.VoiceSound)))
                ]
            elif level == DiagnosticStatus.WARN:
                status.message = "Sound device not open yet"
            else:
                status.message = "Can't open sound device. See http://wiki.ros.org/sound_play/Troubleshooting"
            diag_array = DiagnosticArray()
            diag_array.status.append(status)
            diag_array.header.stamp = rospy.Time.now()
            self.diagnostic_pub.publish(diag_array)
        except Exception as e:
            rospy.logerr("Exception in diagnostics: %s" % str(e))
            rospy.logerr(traceback.format_exc())

if __name__ == '__main__':
    rospy.init_node('sound_play')
    n = SoundPlay()
    rospy.spin()
