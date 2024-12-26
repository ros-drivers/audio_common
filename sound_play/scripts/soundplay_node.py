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
import sys
import tempfile
import threading
import time
import traceback

from ament_index_python.packages import get_package_share_directory
import rclpy.action
import rclpy.duration
import rclpy.logging
import rclpy.node

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from sound_play.action import SoundRequest as SoundRequestAction
from sound_play.msg import SoundRequest


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
    rclpy.logging.get_logger('sound_play').fatal(str)
    exit(1)


class SoundType(object):
    STOPPED = 0
    LOOPING = 1
    COUNTING = 2

    def __init__(self, node, file, device, volume=1.0):
        self.node = node
        self.lock = threading.RLock()
        self.state = self.STOPPED
        self.sound = Gst.ElementFactory.make("playbin", None)
        if self.sound is None:
            raise Exception("Could not create sound player")

        if device:
            self.sink = Gst.ElementFactory.make("alsasink", "sink")
            self.sink.set_property("device", device)
            self.sound.set_property("audio-sink", self.sink)

        if (":" in file):
            uri = file
        elif os.path.isfile(file):
            uri = "file://" + os.path.abspath(file)
        else:
            self.node.get_logger().error('Error: URI is invalid: %s' % file)

        self.uri = uri
        self.volume = volume
        self.sound.set_property('uri', uri)
        self.sound.set_property("volume", volume)
        self.staleness = 1
        self.file = file

        self.bus = self.sound.get_bus()
        self.bus.add_signal_watch()
        self.bus_conn_id = self.bus.connect("message", self.on_stream_end)

    def on_stream_end(self, bus, message):
        if message.type == Gst.MessageType.EOS:
            if (self.state == self.LOOPING):
                self.sound.seek_simple(Gst.Format.TIME, Gst.SeekFlags.FLUSH, 0)
            else:
                self.stop()

    def __del__(self):
        self.destroy_timer(self.timer)
        self.dispose()

    def update(self):
        if self.bus is not None:
            self.bus.poll(Gst.MessageType.ERROR, 10)

    def loop(self):
        self.lock.acquire()
        try:
            self.staleness = 0
            if self.state == self.COUNTING:
                self.stop()

            if self.state == self.STOPPED:
                self.sound.seek_simple(Gst.Format.TIME, Gst.SeekFlags.FLUSH, 0)
                self.sound.set_state(Gst.State.PLAYING)
            self.state = self.LOOPING
        finally:
            self.lock.release()

    def dispose(self):
        self.lock.acquire()
        try:
            if self.bus is not None:
                self.sound.set_state(Gst.State.NULL)
                self.bus.disconnect(self.bus_conn_id)
                self.bus.remove_signal_watch()
                self.bus = None
                self.sound = None
                self.sink = None
                self.state = self.STOPPED
        except Exception as e:
            self.node.get_logger().error(
                'Exception in dispose: %s' % str(e))
        finally:
            self.lock.release()

    def stop(self):
        if self.state != self.STOPPED:
            self.lock.acquire()
            try:
                self.sound.set_state(Gst.State.NULL)
                self.state = self.STOPPED
            finally:
                self.lock.release()

    def single(self):
        self.lock.acquire()
        try:
            self.node.get_logger().debug("Playing %s" % self.uri)
            self.staleness = 0
            if self.state == self.LOOPING:
                self.stop()

            self.sound.seek_simple(Gst.Format.TIME, Gst.SeekFlags.FLUSH, 0)
            self.sound.set_state(Gst.State.PLAYING)
            self.state = self.COUNTING
        finally:
            self.lock.release()

    def command(self, cmd):
        if cmd == SoundRequest.PLAY_STOP:
            self.stop()
        elif cmd == SoundRequest.PLAY_ONCE:
            self.single()
        elif cmd == SoundRequest.PLAY_START:
            self.loop()

    def get_staleness(self):
        self.lock.acquire()
        position = 0
        duration = 0
        try:
            position = self.sound.query_position(Gst.Format.TIME)[1]
            duration = self.sound.query_duration(Gst.Format.TIME)[1]
        except Exception:
            position = 0
            duration = 0
        finally:
            self.lock.release()

        if position != duration:
            self.staleness = 0
        else:
            self.staleness = self.staleness + 1
        return self.staleness

    def get_playing(self):
        return self.state == self.COUNTING


class SoundPlayNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('sound_play')
        Gst.init(None)

        # Start gobject thread to receive gstreamer messages
        GObject.threads_init()
        self.g_loop = threading.Thread(target=GObject.MainLoop().run)
        self.g_loop.daemon = True
        self.g_loop.start()

        self.declare_parameter('loop_rate', 100)
        self.declare_parameter('device', 'default')
        self.declare_parameter(
            'default_voice', 'voice_kal_diphone')
        self.loop_rate = self.get_parameter('loop_rate').value
        self.device = self.get_parameter('device').value
        self.default_voice = self.get_parameter('default_voice').value

        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 1)
        rootdir = os.path.join(
            get_package_share_directory('sound_play'), 'sounds')

        self.builtinsoundparams = {
            SoundRequest.BACKINGUP:
            (os.path.join(rootdir, 'BACKINGUP.ogg'), 0.1),
            SoundRequest.NEEDS_UNPLUGGING:
            (os.path.join(rootdir, 'NEEDS_UNPLUGGING.ogg'), 1),
            SoundRequest.NEEDS_PLUGGING:
            (os.path.join(rootdir, 'NEEDS_PLUGGING.ogg'), 1),
            SoundRequest.NEEDS_UNPLUGGING_BADLY:
            (os.path.join(rootdir, 'NEEDS_UNPLUGGING_BADLY.ogg'), 1),
            SoundRequest.NEEDS_PLUGGING_BADLY:
            (os.path.join(rootdir, 'NEEDS_PLUGGING_BADLY.ogg'), 1),
        }

        self.no_error = True
        self.initialized = False
        self.active_sounds = 0

        self.mutex = threading.Lock()
        self.sub = self.create_subscription(
            SoundRequest, "robotsound", self.callback, 10)
        self._as = rclpy.action.ActionServer(
            self, SoundRequestAction, 'sound_play',
            execute_callback=self.execute_cb,
            handle_accepted_callback=self.handle_accepted_cb)

        # For ros startup race condition
        self.sleep(0.5)
        self.diagnostics(1)

        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._execute_lock = threading.Lock()
        self.timer = self.create_timer(0.1, self.spin_once)

    def spin_once(self):
        self.init_vars()
        self.no_error = True
        self.initialized = True
        with self.mutex:
            try:
                self.idle_loop()
            except Exception as e:
                self.get_logger().error(
                    'Exception in idle_loop: %s' % str(e))
        self.diagnostics(2)

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
                    self.get_logger().debug(
                        'command for uncached wave: "%s"' % data.arg)
                    try:
                        self.filesounds[data.arg] = SoundType(
                            self, data.arg, self.device, data.volume)
                    except Exception:
                        self.get_logger().error(
                            'Error setting up to play "%s".'
                            'Does this file exist on the machine '
                            'on which sound_play is running?'
                            % data.arg)
                        return
                else:
                    self.get_logger().debug(
                        'command for cached wave: "%s"' % data.arg)
                    wav_volume = self.filesounds[data.arg].sound.get_property(
                        'volume')
                    if wav_volume != data.volume:
                        self.get_logger().debug(
                            'volume for cached wave has changed,'
                            'resetting volume')
                        self.filesounds[data.arg].sound.set_property(
                            'volume', data.volume)
                sound = self.filesounds[data.arg]
            else:
                absfilename = os.path.join(
                    get_package_share_directory(data.arg2), data.arg)
                if absfilename not in self.filesounds.keys():
                    self.get_logger().debug(
                        'command for uncached wave: "%s"' % absfilename)
                    try:
                        self.filesounds[absfilename] = SoundType(
                            self, absfilename, self.device, data.volume)
                    except Exception:
                        self.get_logger().error(
                            'Error setting up to play "%s" from package "%s"'
                            'Does this file exist on the machine '
                            'on which sound_play is running?'
                            % (data.arg, data.arg2))
                        return
                else:
                    self.get_logger().debug(
                        'command for cached wave: "%s"' % absfilename)
                    abs_volume = \
                        self.filesounds[absfilename].sound.get_property(
                            'volume')
                    if abs_volume != data.volume:
                        self.get_logger().debug(
                            'volume for cached wave has changed,'
                            'resetting volume')
                        self.filesounds[absfilename].sound.set_property(
                            'volume', data.volume)
                sound = self.filesounds[absfilename]
        elif data.sound == SoundRequest.SAY:
            voice_key = data.arg + '---' + data.arg2
            if voice_key not in self.voicesounds.keys():
                self.get_logger().debug(
                    'command for uncached text: "%s"' % voice_key)
                txtfile = tempfile.NamedTemporaryFile(
                    prefix='sound_play', suffix='.txt')
                (wavfile, wavfilename) = tempfile.mkstemp(
                    prefix='sound_play', suffix='.wav')
                txtfilename = txtfile.name
                os.close(wavfile)
                if data.arg2 == '':
                    voice = self.default_voice
                else:
                    voice = data.arg2
                try:
                    try:
                        if hasattr(data.arg, 'decode'):
                            txtfile.write(
                                data.arg.decode('UTF-8').encode('ISO-8859-15'))
                        else:
                            txtfile.write(data.arg.encode('ISO-8859-15'))
                    except UnicodeEncodeError:
                        if hasattr(data.arg, 'decode'):
                            txtfile.write(data.arg)
                        else:
                            txtfile.write(data.arg.encode('UTF-8'))
                    txtfile.flush()
                    os.system(
                        "text2wave -eval '("+voice+")' "
                        + txtfilename + " -o " + wavfilename)
                    try:
                        if os.stat(wavfilename).st_size == 0:
                            # So we hit the same catch block
                            raise OSError
                    except OSError:
                        self.get_logger().error(
                            'Sound synthesis failed. Is festival installed?'
                            'Is a festival voice installed?'
                            'Try running "rosdep satisfy sound_play|sh".'
                            'Refer to '
                            'http://wiki.ros.org/sound_play/Troubleshooting'
                        )
                        return
                    self.voicesounds[voice_key] = SoundType(
                        self, wavfilename, self.device, data.volume)
                finally:
                    txtfile.close()
            else:
                self.get_logger().debug(
                    'command for cached text: "%s"' % voice_key)
                voice_volume = \
                    self.voicesounds[voice_key].sound.get_property('volume')
                if (voice_volume != data.volume):
                    self.get_logger().debug(
                        'volume for cached text has changed, resetting volume')
                    self.voicesounds[voice_key].sound.set_property(
                        'volume', data.volume)
            sound = self.voicesounds[voice_key]
        else:
            self.get_logger().debug(
                'command for builtin wave: %i' % data.sound)
            if (data.sound not in self.builtinsounds or
                    (data.volume != self.builtinsounds[data.sound].volume
                        and data.sound in self.builtinsounds)):
                params = self.builtinsoundparams[data.sound]
                volume = data.volume
                # use the second param as a scaling for the input volume
                if params[1] != 1:
                    volume = (volume + params[1])/2
                self.builtinsounds[data.sound] = SoundType(
                    self, params[0], self.device, volume)
            sound = self.builtinsounds[data.sound]
        if sound.staleness != 0 and data.command != SoundRequest.PLAY_STOP:
            # This sound isn't counted in active_sounds
            self.get_logger().debug(
                "activating %i %s" % (data.sound, data.arg))
            self.active_sounds = self.active_sounds + 1
            sound.staleness = 0
            #                    if self.active_sounds > self.num_channels:
            #                        mixer.set_num_channels(self.active_sounds)
            #                        self.num_channels = self.active_sounds
        return sound

    def callback(self, data):
        self.get_logger().error('callback: {}'.format(str(data)))
        if not self.initialized:
            return
        with self.mutex:
            try:
                if (data.sound == SoundRequest.ALL
                        and data.command == SoundRequest.PLAY_STOP):
                    self.stopall()
                else:
                    sound = self.select_sound(data)
                    sound.command(data.command)
            except Exception as e:
                self.get_logger().error('Exception in callback: %s' % str(e))
                self.get_logger().info(traceback.format_exc())
            finally:
                self.get_logger().debug("done callback")

    # Purge sounds that haven't been played in a while.
    def cleanupdict(self, dict):
        purgelist = []
        for (key, sound) in iter(dict.items()):
            try:
                staleness = sound.get_staleness()
            except Exception as e:
                self.get_logger().error(
                    'Exception in cleanupdict for sound (%s): %s'
                    % (str(key), str(e)))
                # Something is wrong. Let's purge and try again.
                staleness = 100
            if staleness >= 10:
                purgelist.append(key)
            # Sound is playing
            if staleness == 0:
                self.active_sounds = self.active_sounds + 1
        for key in purgelist:
            self.get_logger().debug('Purging %s from cache' % key)
            # clean up resources
            dict[key].dispose()
            del dict[key]

    def cleanup(self):
        with self.mutex:
            try:
                self.active_sounds = 0
                self.cleanupdict(self.filesounds)
                self.cleanupdict(self.voicesounds)
                self.cleanupdict(self.builtinsounds)
            except Exception:
                self.get_logger().info(
                    'Exception in cleanup: %s' % sys.exc_info()[0])

    def diagnostics(self, state):
        try:
            da = DiagnosticArray()
            ds = DiagnosticStatus()
            ds.name = self.get_name() + ": Node State"
            if state == 0:
                ds.level = DiagnosticStatus.OK
                ds.message = "%i sounds playing" % self.active_sounds
                ds.values.append(
                    KeyValue(
                        key="Active sounds",
                        value=str(self.active_sounds)))
                ds.values.append(
                    KeyValue(
                        key="Allocated sound channels",
                        value=str(self.num_channels)))
                ds.values.append(
                    KeyValue(
                        key="Buffered builtin sounds",
                        value=str(len(self.builtinsounds))))
                ds.values.append(
                    KeyValue(
                        key="Buffered wave sounds",
                        value=str(len(self.filesounds))))
                ds.values.append(
                    KeyValue(
                        key="Buffered voice sounds",
                        value=str(len(self.voicesounds))))
            elif state == 1:
                ds.level = DiagnosticStatus.WARN
                ds.message = "Sound device not open yet."
            else:
                ds.level = DiagnosticStatus.ERROR
                ds.message = "Can't open sound device." + \
                    "See http://wiki.ros.org/sound_play/Troubleshooting"
            da.status.append(ds)
            da.header.stamp = self.get_clock().now().to_msg()
            self.diagnostic_pub.publish(da)
        except Exception as e:
            self.get_logger().error(
                'Exception in diagnostics: %s' % str(e))

    def execute_cb(self, goal_handle):
        data = goal_handle.request.sound_request
        if not self.initialized:
            self.get_logger().error('soundplay_node is not initialized yet.')
            goal_handle.abort()
            return
        with self.mutex:
            # Force only one sound at a time
            self.stopall()
            result = SoundRequestAction.Result()
            try:
                if (data.sound == SoundRequest.ALL
                        and data.command == SoundRequest.PLAY_STOP):
                    self.stopall()
                else:
                    sound = self.select_sound(data)
                    sound.command(data.command)

                    start_time = self.get_clock().now()
                    success = True
                    while sound.get_playing():
                        sound.update()
                        feedback = SoundRequestAction.Feedback()
                        feedback.playing = sound.get_playing()
                        feedback.stamp = (
                            self.get_clock().now() - start_time).to_msg()
                        goal_handle.publish_feedback(feedback)
                        if not goal_handle.is_active:
                            self.get_logger().info(
                                'sound_play action: Preempted')
                            sound.stop()
                            success = False
                            break
                        self.sleep(1.0 / self.loop_rate)
                    if success:
                        result.playing = feedback.playing
                        result.stamp = feedback.stamp
                        self.get_logger().info('sound_play action: Succeeded')
                        goal_handle.succeed()
            except Exception as e:
                goal_handle.abort()
                self.get_logger().error(
                    'Exception in actionlib callback: %s' % str(e))
                self.get_logger().info(traceback.format_exc())
            finally:
                self.get_logger().debug("done actionlib callback")
        return result

    def handle_accepted_cb(self, goal_handle):
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def init_vars(self):
        self.num_channels = 10
        self.builtinsounds = {}
        self.filesounds = {}
        self.voicesounds = {}
        self.hotlist = []
        if not self.initialized:
            self.get_logger().info('sound_play node is ready to play sound')

    def sleep(self, duration):
        time.sleep(duration)

    def get_sound_length(self):
        sound_length = len(self.builtinsounds) \
            + len(self.voicesounds) + len(self.filesounds)
        return sound_length

    def idle_loop(self):
        self.last_activity_time = self.get_clock().now()
        while (self.get_sound_length() > 0 and rclpy.ok()):
            loop_time = self.get_clock().now() - self.last_activity_time
            if loop_time > rclpy.duration.Duration(seconds=10):
                break
            self.diagnostics(0)
            self.sleep(1)
            self.cleanup()


if __name__ == '__main__':
    rclpy.init()
    soundplay_node = SoundPlayNode()
    rclpy.spin(soundplay_node)
    soundplay_node.destroy_node()
    del soundplay_node
    rclpy.shutdown()
