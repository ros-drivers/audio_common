#!/usr/bin/env python
# https://adnanalamkhan.wordpress.com/2015/03/01/using-gstreamer-1-0-with-python/
import gi
# import rospy

gi.require_version('Gst', '1.0')
# gi.require_version('Gtk', '3.0')
# from gi.repository import Gtk
from gi.repository import GObject
from gi.repository import Gst as gst

GObject.threads_init()
gst.init(None)
# rospy.init_node('audio_capture')

# Create the pipeline for our elements.
pipeline = gst.Pipeline()
# Create the elements for our project.

audio_source = gst.ElementFactory.make('filesrc', 'audio_source')
# audio_source = gst.ElementFactory.make('alsasrc', 'audio_source')
decode = gst.ElementFactory.make('mad', 'decode')
convert = gst.ElementFactory.make('audioconvert', 'convert')
equalizer = gst.ElementFactory.make('equalizer-3bands', 'equalizer')
audio_sink = gst.ElementFactory.make('autoaudiosink', 'audio_sink')

# Ensure all elements were created successfully.
if (not pipeline or not audio_source or not decode or
    not convert or not equalizer or not audio_sink):
    print('Not all elements could be created.')
    exit(-1)

# Configure our elements.
filename = 'blah'  # 'Kevin_MacLeod_-_05_-_Impact_Allegretto.mp3'
audio_source.set_property('location', filename)
equalizer.set_property('band1', -24.0)
equalizer.set_property('band2', -24.0)

# Add our elements to the pipeline.
pipeline.add(audio_source)
pipeline.add(decode)
pipeline.add(convert)
pipeline.add(equalizer)
pipeline.add(audio_sink)

# Link our elements together.
audio_source.link(decode)
decode.link(convert)
convert.link(equalizer)
equalizer.link(audio_sink)

# Set our pipelines state to Playing.
# check the following documentation whenever you get
# some AttributeError.
# link: http://lazka.github.io/pgi-docs/#Gst-1.0/flags.html
pipeline.set_state(gst.State.PLAYING)

# Wait until error or EOS.
bus = pipeline.get_bus()

while True:  # not rospy.is_shutdown():
    # msg = bus.timed_pop_filtered(gst.CLOCK_TIME_NONE, gst.MessageType.ERROR | gst.MessageType.EOS)
    msg = bus.timed_pop_filtered(1e9, gst.MessageType.ERROR | gst.MessageType.EOS)
    if msg is None:
        break
    print msg

# Free resources.
pipeline.set_state(gst.State.NULL)
