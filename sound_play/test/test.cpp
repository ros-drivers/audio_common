#include <sound_play/sound_play.h>
#include <unistd.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sound_play_test");

  ros::NodeHandle nh;
  sound_play::SoundHandle h;

  sleep(1);
  
  while(nh.ok())
  {
    h.say("Hello world!");
    sleep(2);

    const char *str1 = "I am annoying.";
    h.repeat(str1);
    sleep(4);
    h.stopsaying(str1);

    h.playwave("/usr/share/xemacs21/xemacs-packages/etc/sounds/boing.wav");
    sleep(2);

    const char *str2 = "/usr/share/xemacs21/xemacs-packages/etc/sounds/piano-beep.wav";
    h.startwave(str2);
    sleep(4);
    h.stopwave(str2);

    h.play(sound_play::SoundRequest::NEEDS_UNPLUGGING);
    sleep(2);

    h.start(sound_play::SoundRequest::BACKINGUP);
    sleep(4);
    h.stop(sound_play::SoundRequest::BACKINGUP);
  }
}
