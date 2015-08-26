#!/usr/bin/env python

#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Willow Garage nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

# Author: Blaise Gassend

import rospy
import roslib
import os, sys
from sound_play.msg import SoundRequest

## \brief Class that publishes messages to the sound_play node.
##
## This class is a helper class for communicating with the sound_play node
## via the \ref sound_play.SoundRequest message. It has two ways of being used:
##
## - It can create Sound classes that represent a particular sound which
##   can be played, repeated or stopped.
##
## - It provides methods for each way in which the sound_play.SoundRequest
##   message can be invoked.

class Sound:
    def __init__(self, client, snd, arg, volume=1.0):
        self.client = client
        self.snd = snd
        self.arg = arg
        self.vol = volume
    
## \brief Play the Sound.
## 
## This method causes the Sound to be played once.

    def play(self):
        self.client.sendMsg(self.snd, SoundRequest.PLAY_ONCE, self.arg, vol=self.vol)

## \brief Play the Sound repeatedly.
##
## This method causes the Sound to be played repeatedly until stop() is
## called.
    
    def repeat(self):
       self.client.sendMsg(self.snd, SoundRequest.PLAY_START, self.arg, vol=self.vol)

## \brief Stop Sound playback.
##
## This method causes the Sound to stop playing.

    def stop(self):
        self.client.sendMsg(self.snd, SoundRequest.PLAY_STOP, self.arg)

## This class is a helper class for communicating with the sound_play node
## via the \ref sound_play.SoundRequest message. There is a one-to-one mapping
## between methods and invocations of the \ref sound_play.SoundRequest message.

class SoundClient:
    def __init__(self):
        self.pub = rospy.Publisher('robotsound', SoundRequest, queue_size=5)

## \brief Create a voice Sound.
##
## Creates a Sound corresponding to saying the indicated text.
##
## \param s Text to say
 
    def voiceSound(self, s, volume=1.0):
        return Sound(self, SoundRequest.SAY, s, volume=volume)

## \brief Create a wave Sound.
##
## Creates a Sound corresponding to indicated file.
##
## \param s File to play. Should be an absolute path that exists on the
## machine running the sound_play node.
    def waveSound(self, sound, volume=1.0):
        if sound[0] != "/":
          rootdir = os.path.join(roslib.packages.get_pkg_dir('sound_play'),'sounds')
          sound = rootdir + "/" + sound
        return Sound(self, SoundRequest.PLAY_FILE, sound, volume=volume)
    
## \brief Create a builtin Sound.
##
## Creates a Sound corresponding to indicated builtin wave.
##
## \param id Identifier of the sound to play.

    def builtinSound(self, id, volume=1.0):
        return Sound(self, id, "", volume)

## \brief Say a string
## 
## Send a string to be said by the sound_node. The vocalization can be
## stopped using stopSaying or stopAll.
## 
## \param text String to say

    def say(self,text, voice='', volume=1.0):
        self.sendMsg(SoundRequest.SAY, SoundRequest.PLAY_ONCE, text, voice, volume)

## \brief Say a string repeatedly
## 
## The string is said repeatedly until stopSaying or stopAll is used.
## 
## \param text String to say repeatedly

    def repeat(self,text, volume=1.0):
        self.sendMsg(SoundRequest.SAY, SoundRequest.PLAY_START, text, vol=volume)

## \brief Stop saying a string
## 
## Stops saying a string that was previously started by say or repeat. The
## argument indicates which string to stop saying.
## 
## \param text Same string as in the say or repeat command

    def stopSaying(self,text):
        self.sendMsg(SoundRequest.SAY, SoundRequest.PLAY_STOP, text)
    
## \brief Plays a WAV or OGG file
## 
## Plays a WAV or OGG file once. The playback can be stopped by stopWave or
## stopAll.
## 
## \param sound Filename of the WAV or OGG file. Must be an absolute path valid
## on the computer on which the sound_play node is running

    def playWave(self, sound, volume=1.0):
        if sound[0] != "/":
          rootdir = os.path.join(roslib.packages.get_pkg_dir('sound_play'),'sounds')
          sound = rootdir + "/" + sound
        self.sendMsg(SoundRequest.PLAY_FILE, SoundRequest.PLAY_ONCE, sound, vol=volume)
    
## \brief Plays a WAV or OGG file repeatedly
## 
## Plays a WAV or OGG file repeatedly until stopWave or stopAll is used.
## 
## \param sound Filename of the WAV or OGG file. Must be an absolute path valid
## on the computer on which the sound_play node is running.

    def startWave(self, sound, volume=1.0):
        if sound[0] != "/":
          rootdir = os.path.join(roslib.packages.get_pkg_dir('sound_play'),'sounds')
          sound = rootdir + "/" + sound
        self.sendMsg(SoundRequest.PLAY_FILE, SoundRequest.PLAY_START, sound, vol=volume)

##  \brief Stop playing a WAV or OGG file
## 
## Stops playing a file that was previously started by playWave or
## startWave.
## 
## \param sound Same string as in the playWave or startWave command

    def stopWave(self,sound):
        if sound[0] != "/":
          rootdir = os.path.join(roslib.package.get_pkg_dir('sound_play'),'sounds')
          sound = rootdir + "/" + sound
        self.sendMsg(SoundRequest.PLAY_FILE, SoundRequest.PLAY_STOP, sound)

## \brief Plays a WAV or OGG file
## 
## Plays a WAV or OGG file once. The playback can be stopped by stopWaveFromPkg or
## stopAll.
## 
## \param package Package name containing the sound file.
## \param sound Filename of the WAV or OGG file. Must be an path relative to the package valid
## on the computer on which the sound_play node is running

    def playWaveFromPkg(self, package, sound, volume=1.0):
        self.sendMsg(SoundRequest.PLAY_FILE, SoundRequest.PLAY_ONCE, sound, package, volume)

## \brief Plays a WAV or OGG file repeatedly
## 
## Plays a WAV or OGG file repeatedly until stopWaveFromPkg or stopAll is used.
## 
## \param package Package name containing the sound file.
## \param sound Filename of the WAV or OGG file. Must be an path relative to the package valid
## on the computer on which the sound_play node is running

    def startWaveFromPkg(self, package, sound, volume=1.0):
        self.sendMsg(SoundRequest.PLAY_FILE, SoundRequest.PLAY_START, sound, package, volume)

##  \brief Stop playing a WAV or OGG file
## 
## Stops playing a file that was previously started by playWaveFromPkg or
## startWaveFromPkg.
## 
## \param package Package name containing the sound file.
## \param sound Filename of the WAV or OGG file. Must be an path relative to the package valid
## on the computer on which the sound_play node is running

    def stopWaveFromPkg(self,sound, package):
        self.sendMsg(SoundRequest.PLAY_FILE, SoundRequest.PLAY_STOP, sound, package)

## \brief Play a buildin sound
##
## Starts playing one of the built-in sounds. built-ing sounds are documented
## in \ref SoundRequest.msg. Playback can be stopped by stopall.
##
## \param sound Identifier of the sound to play.

    def play(self,sound, volume=1.0):
        self.sendMsg(sound, SoundRequest.PLAY_ONCE, "", vol=volume)

## \brief Play a buildin sound repeatedly
##
## Starts playing one of the built-in sounds repeatedly until stop or
## stopall is used. Built-in sounds are documented in \ref SoundRequest.msg.
##
## \param sound Identifier of the sound to play.
    
    def start(self,sound, volume=1.0):
        self.sendMsg(sound, SoundRequest.PLAY_START, "", vol=volume)

## \brief Stop playing a built-in sound
##
## Stops playing a built-in sound started with play or start. 
##
## \param sound Same sound that was used to start playback
    
    def stop(self,sound):
        self.sendMsg(sound, SoundRequest.PLAY_STOP, "")

## \brief Stop all currently playing sounds
##
## This method stops all speech, wave file, and built-in sound playback.
  
    def stopAll(self):
        self.stop(SoundRequest.ALL)

    def sendMsg(self, snd, cmd, s, arg2="", vol=1.0):
        msg = SoundRequest()
        msg.sound = snd
        
        if vol < 0:
            msg.volume = 0
        elif vol > 1.0:
            msg.volume = 1.0
        else:
            msg.volume = vol
        
        msg.command = cmd
        msg.arg = s
        msg.arg2 = arg2 
        self.pub.publish(msg)

        if self.pub.get_num_connections() < 1:
            rospy.logwarn("Sound command issued, but no node is subscribed to the topic. Perhaps you forgot to run soundplay_node.py?");
