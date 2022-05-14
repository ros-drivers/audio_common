import os
import tempfile

import rospy

from sound_play.sound_play_plugin import SoundPlayPlugin


class FestivalPlugin(SoundPlayPlugin):

    _default_voice = 'voice_kal_diphone'

    def __init__(self):
        super(FestivalPlugin, self).__init__()

    def sound_play_say_plugin(self, text, voice):
        if voice is None or voice == '':
            voice = self._default_voice
        txtfile = tempfile.NamedTemporaryFile(
            prefix='sound_play', suffix='.txt')
        (wavfile, wavfilename) = tempfile.mkstemp(
            prefix='sound_play', suffix='.wav')
        txtfilename = txtfile.name
        os.close(wavfile)
        try:
            try:
                if hasattr(text, 'decode'):
                    txtfile.write(
                        text.decode('UTF-8').encode('ISO-8859-15'))
                else:
                    txtfile.write(
                        text.encode('ISO-8859-15'))
            except UnicodeEncodeError:
                if hasattr(text, 'decode'):
                    txtfile.write(text)
                else:
                    txtfile.write(text.encode('UTF-8'))
            txtfile.flush()
            cmd = "text2wave -eval '({0})' {1} -o {2}".format(
                voice, txtfilename, wavfilename)
            os.system(cmd)
            try:
                if os.stat(wavfilename).st_size == 0:
                    # So we hit the same catch block
                    raise OSError
            except OSError:
                rospy.logerr(
                    'Sound synthesis failed.'
                    'Is festival installed?'
                    'Is a festival voice installed?'
                    'Try running "rosdep satisfy sound_play|sh".'
                    'Refer to http://wiki.ros.org/'
                    'sound_play/Troubleshooting'
                )
                return None
        finally:
            txtfile.close()
        return wavfilename
