#!/usr/bin/env python

#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2008, Willow Garage, Inc.
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
from sound_play.msg import SoundRequest

## This class is a helper class for communicating with the sound_play node
## via the \ref sound_play::SoundRequest message. There is a one-to-one mapping
## between methods and invocations of the \ref sound_play::SoundRequest message.

class SoundHandle:
    def __init__(self):
        self.pub = rospy.Publisher('robotsound', SoundRequest)

## \brief Say a string
## 
## Send a string to be said by the sound_node. The vocalization can be
## stopped using stopsaying or stopall.
## 
## \param text String to say

    def say(self,text):
        msg = SoundRequest()
        msg.sound = SoundRequest.SAY
        msg.command = SoundRequest.PLAY_ONCE
        msg.arg=text
        self.pub.publish(msg)

## \brief Say a string repeatedly
## 
## The string is said repeatedly until stopsaying or stopall is used.
## 
## \param text String to say repeatedly

    def repeat(self,text):
        msg = SoundRequest()
        msg.sound = SoundRequest.SAY
        msg.command = SoundRequest.PLAY_START
        msg.arg=text
        self.pub.publish(msg)

## \brief Stop saying a string
## 
## Stops saying a string that was previously started by say or repeat. The
## argument indicates which string to stop saying.
## 
## \param text Same string as in the say or repeat command

    def stopsaying(self,text):
        msg = SoundRequest()
        msg.sound = SoundRequest.SAY
        msg.command = SoundRequest.PLAY_STOP
        msg.arg=text
        self.pub.publish(msg)

## \brief Plays a WAV or OGG file
## 
## Plays a WAV or OGG file once. The playback can be stopped by stopwave or
## stopall.
## 
## \param sound Filename of the WAV or OGG file. Must be an absolute path valid
## on the computer on which the sound_play node is running

    def playwave(self,sound):
        msg = SoundRequest()
        msg.sound = SoundRequest.PLAY_FILE
        msg.command = SoundRequest.PLAY_ONCE
        msg.arg=sound
        self.pub.publish(msg)

## \brief Plays a WAV or OGG file repeatedly
## 
## Plays a WAV or OGG file repeatedly until stopwave or stopall is used.
## 
## \param sound Filename of the WAV or OGG file. Must be an absolute path valid
## on the computer on which the sound_play node is running.

    def startwave(self,sound):
        msg = SoundRequest()
        msg.sound = SoundRequest.PLAY_FILE
        msg.command = SoundRequest.PLAY_START
        msg.arg=sound
        self.pub.publish(msg)

##  \brief Stop playing a WAV or OGG file
## 
## Stops playing a file that was previously started by playwave or
## startwave.
## 
## \param sound Same string as in the playwave or startwave command

    def stopwave(self,sound):
        msg = SoundRequest()
        msg.sound = SoundRequest.PLAY_FILE
        msg.command = SoundRequest.PLAY_STOP
        msg.arg=sound
        self.pub.publish(msg)

## \brief Play a buildin sound
##
## Starts playing one of the built-in sounds. built-ing sounds are documented
## in \ref SoundRequest.msg. Playback can be stopped by stopall.
##
## \param sound Identifier of the sound to play.

    def play(self,sound):
        msg = SoundRequest()
        msg.sound = sound
        msg.command = SoundRequest.PLAY_ONCE
        self.pub.publish(msg)

## \brief Play a buildin sound repeatedly
##
## Starts playing one of the built-in sounds repeatedly until stop or
## stopall is used. Built-in sounds are documented in \ref SoundRequest.msg.
##
## \param sound Identifier of the sound to play.
    
    def start(self,sound):
        msg = SoundRequest()
        msg.sound = sound
        msg.command = SoundRequest.PLAY_START
        self.pub.publish(msg)

## \brief Stop playing a built-in sound
##
## Stops playing a built-in sound started with play or start. 
##
## \param sound Same sound that was used to start playback
    
    def stop(self,sound):
        msg = SoundRequest()
        msg.sound = sound
        msg.command = SoundRequest.PLAY_STOP
        self.pub.publish(msg)

## \brief Stop all currently playing sounds
##
## This method stops all speech, wave file, and built-in sound playback.
  
    def stopall(self):
        self.stop(SoundRequest.ALL)

