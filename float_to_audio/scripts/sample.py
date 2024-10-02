#!/usr/bin/env python
# Lucas Walter
# When trigger record a ChannelFloat32 stream, appending the sequence of messages to
# a single ChannelFloat32 as need until recording is stopped.
# Then when trigger play the sample on an output ChannelFloat32 topic, optionally
# resample it to play at different rates.
