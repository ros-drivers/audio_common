Header header      # Header timestamp should be acquisition time of audio
                   # Header frame_id should be position of microphone

uint8 channels     # Number of channels
uint32 sample_rate # Sampling rate [Hz]
uint8 depth        # Number of bits per audio data

uint8[] data       # Audio data buffer
                   # WAVE format is assumed
