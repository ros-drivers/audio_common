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

from std_srvs.srv import Empty, EmptyResponse
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
    err_string = """**************************************************************
Error opening pygst. Is gstreamer installed? (sudo apt-get install python-gst0.10
**************************************************************
"""
    rospy.logfatal(err_string)
    print err_string
    exit(1)


class SoundObject(object):
    def __init__(self, uri, volume):
        self.once = False
        self.last_play_time = None
        self.sound = gst.element_factory_make('playbin2', 'player')
        self.uri = uri
        self.volume = volume

        self.bus = self.sound.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect('message', self.stop_callback)

        self.lock = RLock()
    def __del__(self):
        self.stop()
    @property
    def uri(self):
        return self.sound.get_property('uri')
    @uri.setter
    def uri(self, value):
        self.sound.set_property('uri', value)
    @property
    def volume(self):
        return self.sound.get_property('volume')
    @volume.setter
    def volume(self, value):
        self.sound.set_property('volume', value)
    def is_playing(self):
        return self.sound.get_state() == gst.STATE_PLAYING
    def stop_callback(self, bus, msg):
        if msg.type == gst.MESSAGE_EOS:
            self.last_play_time = rospy.Time.now()
            rospy.logdebug('[%s] reached to end of file' % self.uri)
            if self.once:
                self.stop()
            else:
                self.play(once=False)
        elif msg.type == gst.MESSAGE_ERROR:
            err, debug = msg.parse_error()
            rospy.logerr("[%s] error on gstreamer: %s (debug: %s)" % (self.uri, err, debug))
            self.stop()
    def poll(self):
        self.bus.poll(gst.MESSAGE_ERROR, 10)
    def stop(self):
        with self.lock:
            self.sound.set_state(gst.STATE_NULL)
    def play(self, once=True):
        with self.lock:
            self.once = once
            self.sound.seek_simple(gst.FORMAT_TIME, gst.SEEK_FLAG_FLUSH, 0)
            self.sound.set_state(gst.STATE_PLAYING)
            rospy.logdebug('[%s] play once=%s' % (self.uri, once))

class SoundLoader(object):
    sounds_dir = os.path.join(get_pkg_dir('sound_play'), 'sounds')
    builtin_sound_params = {
        SoundRequest.BACKINGUP              : (os.path.join(sounds_dir, 'BACKINGUP.ogg'), 0.1),
        SoundRequest.NEEDS_UNPLUGGING       : (os.path.join(sounds_dir, 'NEEDS_UNPLUGGING.ogg'), 1),
        SoundRequest.NEEDS_PLUGGING         : (os.path.join(sounds_dir, 'NEEDS_PLUGGING.ogg'), 1),
        SoundRequest.NEEDS_UNPLUGGING_BADLY : (os.path.join(sounds_dir, 'NEEDS_UNPLUGGING_BADLY.ogg'), 1),
        SoundRequest.NEEDS_PLUGGING_BADLY   : (os.path.join(sounds_dir, 'NEEDS_PLUGGING_BADLY.ogg'), 1),
    }
    def __init__(self, cache_saying=False, file_prefix='sound_play'):
        self.sounds = {}
        self.cache_saying = cache_saying
        self.file_prefix = file_prefix
    def clear_cache(self, cache_timeout_sec=600.0):
        deleted_sound_num = 0
        for key, sound in self.sounds.items():
            if sound.last_play_time is not None:
                from_last_play = (rospy.Time.now() - sound.last_play_time).to_sec()
                if from_last_play > cache_timeout_sec:
                    try:
                        sound.stop()
                        del self.sounds[key]
                        deleted_sound_num += 1
                        rospy.logdebug('deleted cache: %s' % key)
                    except Exception as e:
                        rospy.logerr('Failed to clear cache %s' % key)
        if deleted_sound_num > 0:
            rospy.loginfo('deleted %d cached sounds' % deleted_sound_num)

    def poll(self):
        rospy.logdebug('current sound objects: %s' % self.sounds.keys())
        for sound in self.sounds.values():
            sound.poll()
    def stop_all(self):
        for sound in self.sounds.values():
            sound.stop()
    def uri_from_path(self, path):
        if ':' in path:
            return path
        elif os.path.isfile(path):
            return 'file://' + os.path.abspath(path)
        else:
            raise Exception('Invalid URI: %s' % path)
    def load_from_file_path(self, path, volume=1.0):
        uri = self.uri_from_path(path)
        if not uri in self.sounds:
            self.sounds[uri] = SoundObject(uri=uri, volume=volume)
        return self.sounds[uri]
    def load_from_builtin_sound(self, sound):
        if not sound in self.builtin_sound_params:
            raise Exception('sound type %d is not in builtin_sound_prams' % sound)
        path, volume = self.builtin_sound_params[sound]
        return self.load_from_file_path(path, volume)
    def load_from_ros_path(self, pkg, rel_path, volume=1.0):
        file_path = os.path.join(get_pkg_dir(pkg), rel_path)
        return self.load_from_file_path(file_path, volume)
    def load_from_speech_message(self, txt, voice, volume=1.0):
        if not txt in self.sounds:
            if self.cache_saying:
                target_wav_path = os.path.join(tempfile.gettempdir(), self.file_prefix + txt + '.wav')
            else:
                with tempfile.NamedTemporaryFile(prefix=self.file_prefix, suffix='.wav') as f:
                    target_wav_path = f.name
            if not os.path.exists(target_wav_path) or os.stat(target_wav_path).st_size == 0:
                with tempfile.NamedTemporaryFile(prefix=self.file_prefix, suffix='.txt') as txt_file:
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
                target_wav_permission = os.stat(target_wav_path).st_mode
                if bool(target_wav_permission & stat.S_IWGRP) is False:
                    # give write permission to group
                    os.chmod(target_wav_path, target_wav_permission | stat.S_IWGRP)
            uri = self.uri_from_path(target_wav_path)
            self.sounds[txt] = SoundObject(uri=uri, volume=volume)
        return self.sounds[txt]

class SoundPlay(object):
    def __init__(self):
        self.lock = Lock()
        self.cache_timeout_sec = rospy.get_param('~cache_timeout', 600.0)
        self.cache_saying_file = rospy.get_param('~cache_saying_file', False)
        self.sound_file_prefix = rospy.get_param('~sound_file_prefix', 'sound_play')
        self.update_rate = rospy.get_param('~update_rate', 5.0)
        self.loader = SoundLoader(cache_saying=self.cache_saying_file, file_prefix=self.sound_file_prefix)
        self.sound_request_sub = rospy.Subscriber('robotsound', SoundRequest, self.sound_request_callback)
        self.clear_cache_srv = rospy.Service('~clear_cache', Empty, self.clear_cache)
        self.action_server = actionlib.SimpleActionServer('robotsound', SoundRequestAction,
                                                          execute_cb=self.sound_request_action_execute_cb,
                                                          auto_start=False)
        self.action_server.start()
        self.poll_timer = rospy.Timer(rospy.Duration(1.0 / self.update_rate), self.poll)
    def poll(self, event=None):
        self.loader.poll()
        self.loader.clear_cache(self.cache_timeout_sec)
    def clear_cache(self, req=None):
        self.loader.clear_cache(0.0)
        return EmptyResponse()
    def command_from_request(self, msg):
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
    def sound_request_callback(self, msg):
        with self.lock:
            try:
                sound = self.command_from_request(msg)
            except Exception as e:
                rospy.logerr('Failed to command request: %s' % str(e))
                rospy.logerr(traceback.format_exc())
    def sound_request_action_execute_cb(self, goal):
        req = goal.sound_request
        with self.lock:
            try:
                start_time = rospy.Time.now()
                sound = self.command_from_request(req)
                feedback_msg = SoundRequestFeedback()
                r = rospy.Rate(self.update_rate)
                while sound.is_playing():
                    if self.action_server.is_preempt_requested():
                        rospy.loginfo('sound_play action: Preempted')
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
                rospy.loginfo('sound_play action: Succeeded')
            except Exception as e:
                rospy.logerr('Exception in sound_play action callback: %s' % str(e))
                rospy.logerr(traceback.format_exc())

if __name__ == '__main__':
    rospy.init_node('sound_play')
    n = SoundPlay()
    rospy.spin()
