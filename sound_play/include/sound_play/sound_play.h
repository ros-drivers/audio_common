#ifndef __SOUND_PLAY__H__
#define __SOUND_PLAY__H__

#include <string>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sound_play/SoundRequest.h>

namespace sound_play
{

/** 
 * This class is a helper class for communicating with the sound_play node
 * via the \ref sound_play::SoundRequest message. There is a one-to-one mapping
 * between methods and invocations of the \ref sound_play::SoundRequest message.
 */

class SoundHandle
{
public:
  SoundHandle()
  {
    pub_ = ros::NodeHandle().advertise<sound_play::SoundRequest>("/robotsound", 1);
  }

/** \brief Say a string
 *
 * Send a string to be said by the sound_node. The vocalization can be
 * stopped using stopsaying or stopall.
 *
 * \param s String to say
 */
	
	void say(const std::string s)
  {
    sendmsg(SoundRequest::SAY, SoundRequest::PLAY_ONCE, s);
  }

/** \brief Say a string repeatedly
 *
 * The string is said repeatedly until stopsaying or stopall is used.
 *
 * \param s String to say repeatedly
 */
	
  void repeat(const std::string s)
  {
    sendmsg(SoundRequest::SAY, SoundRequest::PLAY_START, s);
  }

/** \brief Stop saying a string
 *
 * Stops saying a string that was previously started by say or repeat. The
 * argument indicates which string to stop saying.
 *
 * \param s Same string as in the say or repeat command
 */

  void stopsaying(const std::string s)
  {
    sendmsg(SoundRequest::SAY, SoundRequest::PLAY_STOP, s);
  }

/** \brief Plays a WAV or OGG file
 *
 * Plays a WAV or OGG file once. The playback can be stopped by stopwave or
 * stopall.
 *
 * \param s Filename of the WAV or OGG file. Must be an absolute path valid
 * on the computer on which the sound_play node is running
 */

  void playwave(const std::string s)
  {
    sendmsg(SoundRequest::PLAY_FILE, SoundRequest::PLAY_ONCE, s);
  }

/** \brief Plays a WAV or OGG file repeatedly
 *
 * Plays a WAV or OGG file repeatedly until stopwave or stopall is used.
 *
 * \param s Filename of the WAV or OGG file. Must be an absolute path valid
 * on the computer on which the sound_play node is running.
 */

  void startwave(const std::string s)
  {
    sendmsg(SoundRequest::PLAY_FILE, SoundRequest::PLAY_START, s);
  }

/** \brief Stop playing a WAV or OGG file
 *
 * Stops playing a file that was previously started by playwave or
 * startwave.
 *
 * \param s Same string as in the playwave or startwave command
 */

  void stopwave(const std::string s)
  {
    sendmsg(SoundRequest::PLAY_FILE, SoundRequest::PLAY_STOP, s);
  }

/** \brief Play a buildin sound
 *
 * Starts playing one of the built-in sounds. built-ing sounds are documented
 * in \ref SoundRequest.msg. Playback can be stopped by stopall.
 *
 * \param sound Identifier of the sound to play.
 */
	
	void play(int sound)
  {
    sendmsg(sound, SoundRequest::PLAY_ONCE);
  }

/** \brief Play a buildin sound repeatedly
 *
 * Starts playing one of the built-in sounds repeatedly until stop or stopall 
 * is used. Built-in sounds are documented in \ref SoundRequest.msg.
 *
 * \param sound Identifier of the sound to play.
 */
	
	void start(int sound) 
	{ 
		sendmsg(sound, SoundRequest::PLAY_START); 
	}

/** \brief Stop playing a built-in sound
 *
 * Stops playing a built-in sound started with play or start.
 *
 * \param sound Same sound that was used to start playback.
 */ 

  void stop(int sound)
  {
    sendmsg(sound, SoundRequest::PLAY_STOP);
  }

/** \brief Stop all currently playing sounds
 *
 * This method stops all speech, wave file, and built-in sound playback.
 */

  void stopall()
	{
		stop(SoundRequest::ALL);
	}

private:
  void sendmsg(int snd, int cmd, std::string s = "")
  {
    SoundRequest msg;
    msg.sound = snd;
    msg.command = cmd;
    msg.arg = s;
    pub_.publish(msg);
  }
  
  ros::Publisher pub_;
};
};

#endif
