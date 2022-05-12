class SoundPlayPlugin(object):

    """Base class for sound_play plugin

    This is a base class for sound_play plugin.
    sound_play plugin has one method; sound_play_say_plugin.
    sound_play_start_plugin run when command is SAY.

    sound_play plugin is defined in yaml format as below;
    - name: sound_play/festival_plugin  # plugin name
      module: sound_play.festival_plugin.FestivalPlugin
      # plugin module name

    Also, sound_play plugin yaml file is exported in package.xml as below;
     <export>
       <sound_play plugin="${prefix}/sound_play_plugin.yaml" />
     </export>
    """

    def __init__(self):
        pass

    def sound_play_say_plugin(self, text, voice):
        """Start plugin for sound_play

        Args:
            text (string): speech text
            voice (string): speech voice
        """
        return None
