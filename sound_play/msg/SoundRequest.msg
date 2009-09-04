# IMPORTANT: You should never have to generate this message yourself.
# Use the sound_play::SoundHandle C++ helper or the
# sound_play.libsoundplay.SoundHandle Python helper.

# Sounds
byte BACKINGUP = 1
byte NEEDS_UNPLUGGING = 2
byte NEEDS_PLUGGING = 3
byte NEEDS_UNPLUGGING_BADLY = 4
byte NEEDS_PLUGGING_BADLY = 5

# Sound identifiers that have special meaning
byte ALL = -1 # Only legal with PLAY_STOP
byte PLAY_FILE = -2
byte SAY = -3

byte sound # Selects which sound to play (see above)

# Commands
byte PLAY_STOP = 0 # Stop this sound from playing
byte PLAY_ONCE = 1 # Play the sound once
byte PLAY_START = 2 # Play the sound in a loop until a stop request occurs

byte command # Indicates what to do with the sound

string arg # file name or text to say
