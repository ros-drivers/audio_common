Header header        # Header timestamp should be acquisition time of audio
                     # Header frame_id should be position of microphone

uint8 channels       # Number of channels
uint32 sample_rate   # Sampling rate [Hz]
string sample_format # Format for Compressed audio
                     # Format list can be given by following command
                     # $ gst-inspect-1.0 lamemp3enc | grep format:
uint32 bitrate       # Amount of audio data per second [bits/s]

string format        # Specifies the format of the audio data
                     # For example, mp3
uint8[] data         # Compressed audio buffer
