#!/usr/bin/env python
# Lucas Walter
# Subscribe to ChannelFloat32 topic and mix together messages to provide a
# fixed sample rate output ChannelFloat32 suitable for conversion to audio
# in a float_to_audio node.
# Generate silence as needed when there are no incoming samples
