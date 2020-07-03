Header header        # Header timestamp should be acquisition time of audio
                     # Header frame_id should be position of microphone

uint8 channels       # Number of channels
uint32 sample_rate   # Sampling rate [Hz]
string sample_format # Format for PCM audio
                     # Format list can be given by following command
                     # $ gst-inspect-1.0 wavenc | grep format:

uint8[] data         # Audio data buffer
                     # WAVE format is assumed
