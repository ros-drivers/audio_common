import os
import threading

import rospy

from sound_play.msg import SoundRequest

try:
    import gi
    gi.require_version('Gst', '1.0')
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


class SoundType(object):
    STOPPED = 0
    LOOPING = 1
    COUNTING = 2

    def __init__(self, file, device, volume=1.0):
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
            rospy.logerr('Error: URI is invalid: %s' % file)

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
        # stop our GST object so that it gets garbage-collected
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
            rospy.logerr('Exception in dispose: %s' % str(e))
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
            rospy.logdebug("Playing %s" % self.uri)
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
