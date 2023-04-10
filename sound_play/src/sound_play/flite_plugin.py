import os
import tempfile

import resource_retriever
import rospkg
import rospy

from sound_play.sound_play_plugin import SoundPlayPlugin


class FlitePlugin(SoundPlayPlugin):

    _default_voice = 'kal'

    def __init__(self):
        super(FlitePlugin, self).__init__()
        self._default_voice_path = None

    def get_default_voice_path(self):
        if self._default_voice_path is None:
            self._default_voice_path = os.path.join(
                rospkg.RosPack().get_path('sound_play'),
                'resources/flitevox')
        return self._default_voice_path

    def sound_play_say_plugin(self, text, voice):
        if voice is None or voice == '':
            voice = self._default_voice
        if voice.endswith('.flitevox'):
            if voice.startswith('package://'):
                voice = resource_retriever.get(voice)
            elif voice.startswith('/'):
                voice = voice
            else:
                voice = os.path.join(
                    self.get_default_voice_path(), voice)
        (wavfile, wavfilename) = tempfile.mkstemp(
            prefix='sound_play', suffix='.wav')
        os.close(wavfile)
        cmd = "flite -voice {0} -t \"{1}\" -o {2}".format(
            voice, text, wavfilename)
        os.system(cmd)
        try:
            if os.stat(wavfilename).st_size == 0:
                # So we hit the same catch block
                raise OSError
        except OSError:
            rospy.logerr(
                'Sound synthesis failed.'
                'Is flite installed?'
                'Is a flite voice installed?'
                'Try running "rosdep satisfy sound_play|sh".'
                'Refer to http://wiki.ros.org/'
                'sound_play/Troubleshooting'
            )
            return None
        return wavfilename
