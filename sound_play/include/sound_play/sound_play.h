#ifndef __SOUND_PLAY__SOUND_PLAY__H__
#define __SOUND_PLAY__SOUND_PLAY__H__

#include <string>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sound_play/SoundRequest.h>
#include <boost/thread.hpp>

namespace sound_play
{

/// Using SoundHandle is deprecated. The new name is SoundClient.
#define SoundHandle SoundClient

/**
 * \brief Class that publishes messages to the sound_play node.
 *
 * This class is a helper class for communicating with the sound_play node
 * via the \ref sound_play::SoundRequest message. It has two ways of being used:
 *
 * - It can create Sound classes that represent a particular sound which
 *   can be played, repeated or stopped.
 *
 * - It provides methods for each way in which the sound_play::SoundRequest
 *   message can be invoked.
 */

class SoundClient
{
public:
	class Sound
	{
		friend class SoundClient;
	private:
		int snd_;
		std::string arg_;
		SoundClient *client_;

		Sound(SoundClient *sc, int snd, const std::string &arg)
		{
			client_ = sc;
			snd_ = snd;
			arg_ = arg;
		}

	public:
		/**
		 * \brief Play the Sound.
		 *
		 * This method causes the Sound to be played once.
		 */

		void play()
		{
			client_->sendmsg(snd_, SoundRequest::PLAY_ONCE, arg_);
		}

		/**
		 * \brief Play the Sound repeatedly.
		 *
		 * This method causes the Sound to be played repeatedly until stop() is
		 * called.
		 */

		void repeat()
		{
			client_->sendmsg(snd_, SoundRequest::PLAY_START, arg_);
		}

		/**
		 * \brief Stop Sound playback.
		 *
		 * This method causes the Sound to stop playing.
		 */

		void stop()
		{
			client_->sendmsg(snd_, SoundRequest::PLAY_STOP, arg_);
		}
	};
	
/** \brief Create a SoundClient that publishes on the given topic
 *
 * Creates a SoundClient that publishes to the given topic relative to the
 * given NodeHandle.
 *
 * \param nh Node handle to use when creating the topic.
 *
 * \param topic Topic to publish to.
 */

	SoundClient(ros::NodeHandle &nh, const std::string &topic)
  {
		init(nh, topic);
	}

/** \brief Create a SoundClient with the default topic
 *
 * Creates a SoundClient that publishes to "robotsound".
 */

	SoundClient()
  {
		init(ros::NodeHandle(), "robotsound");
  }

/**
 * \brief Create a voice Sound.
 *
 * Creates a Sound corresponding to saying the indicated text.
 *
 * \param s Text to say
 */

  Sound voiceSound(const std::string &s)
	{
		return Sound(this, SoundRequest::SAY, s);
	}

/**
 * \brief Create a wave Sound.
 *
 * Creates a Sound corresponding to indicated file.
 *
 * \param s File to play. Should be an absolute path that exists on the
 * machine running the sound_play node.
 */

  Sound waveSound(const std::string &s)
	{
		return Sound(this, SoundRequest::PLAY_FILE, s);
	}

/**
 * \brief Create a builtin Sound.
 *
 * Creates a Sound corresponding to indicated builtin wave.
 *
 * \param id Identifier of the sound to play.
 */

  Sound builtinSound(int id)
	{
		return Sound(this, id, "");
	}

/** \brief Say a string
 *
 * Send a string to be said by the sound_node. The vocalization can be
 * stopped using stopsaying or stopall.
 *
 * \param s String to say
 */
	
	void say(const std::string &s)
  {
    sendmsg(SoundRequest::SAY, SoundRequest::PLAY_ONCE, s);
  }

/** \brief Say a string repeatedly
 *
 * The string is said repeatedly until stopsaying or stopall is used.
 *
 * \param s String to say repeatedly
 */
	
  void repeat(const std::string &s)
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

  void stopsaying(const std::string &s)
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

  void playwave(const std::string &s)
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

  void startwave(const std::string &s)
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

  void stopwave(const std::string &s)
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

  /** \brief Turns warning messages on or off.
	 *  
	 * If a message is sent when no node is subscribed to the topic, a
	 * warning message is printed. This method can be used to enable or
	 * disable warnings.
	 *
	 * \param state True to turn off messages, false to turn them on.
	 */

  void setQuiet(bool state)
	{
		quiet_ = state;
	}

private:
	void init(ros::NodeHandle nh, const std::string &topic)
	{
    nh_ = nh;
		pub_ = nh.advertise<sound_play::SoundRequest>(topic, 5);
		quiet_ = false;
  }

	void sendmsg(int snd, int cmd, const std::string &s = "")
  {
		boost::mutex::scoped_lock lock(mutex_);
    
		if (!nh_.ok())
			return;
		
		SoundRequest msg;
    msg.sound = snd;
    msg.command = cmd;
    msg.arg = s;
		pub_.publish(msg);

		if (pub_.getNumSubscribers() == 0 && !quiet_)
			ROS_WARN("Sound command issued, but no node is subscribed to the topic.");
  }

	bool quiet_;
	ros::NodeHandle nh_;
  ros::Publisher pub_;
	boost::mutex mutex_;
};

typedef SoundClient::Sound Sound;
};

#endif
