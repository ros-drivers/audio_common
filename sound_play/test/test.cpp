#include <sound_play/sound_play.h>
#include <unistd.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sound_play_test");

  ros::NodeHandle nh;
  sound_play::SoundClient sc;

  sleep(1);
  
  while(nh.ok())
  {
    sc.say("Hello world!");
    sleep(2);

    const char *str1 = "I am annoying.";
    sc.repeat(str1);
    sleep(4);
    sc.stopsaying(str1);

    sc.playwave("/usr/share/xemacs21/xemacs-packages/etc/sounds/boing.wav");
    sleep(2);

    const char *str2 = "/usr/share/xemacs21/xemacs-packages/etc/sounds/piano-beep.wav";
    sc.startwave(str2);
    sleep(4);
    sc.stopwave(str2);

    sc.play(sound_play::SoundRequest::NEEDS_UNPLUGGING);
    sleep(2);

    sc.start(sound_play::SoundRequest::BACKINGUP);
    sleep(4);
    sc.stop(sound_play::SoundRequest::BACKINGUP);

		sleep(2);
		sound_play::Sound s1 = sc.waveSound("/usr/share/xemacs21/xemacs-packages/etc/sounds/boing.wav");
		s1.repeat();
		sleep(1);
		s1.stop();
		
		sleep(2);
		sound_play::Sound s2 = sc.voiceSound("This is a really long sentence that will get cut off.");
		s2.play();
		sleep(1);
		s2.stop();

		sleep(2);
		sound_play::Sound s3 = sc.builtinSound(sound_play::SoundRequest::NEEDS_UNPLUGGING_BADLY);
		s3.play();
		sleep(1);
		s3.stop();
  }
}
