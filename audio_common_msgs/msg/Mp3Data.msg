Header header        # Header timestamp should be acquisition time of audio
                     # Header frame_id should be position of microphone

uint8 channels       # Number of channels
uint32 sample_rate   # Sampling rate [Hz]
string sample_format # Format for MP3 audio
                     # Format list can be given by following command
                     # $ gst-inspect-1.0 lamemp3enc | grep format:
uint32 bitrate       # Amount of audio data per second [bits/s]

uint8[] data         # MP3 audio buffer
