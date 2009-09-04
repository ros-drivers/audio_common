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

import roslib; roslib.load_manifest('sound_play')

import rospy
import threading
from sound_play.msg import SoundRequest
import os
import logging
import sys
import traceback                                                 

try:
    import pygame.mixer as mixer
except:
    str="""
**************************************************************
Error opening pygame.mixer. Is pygame installed? (sudo apt-get install python-pygame)
**************************************************************
"""
    rospy.logfatal(str)
    print str


class soundtype:
    STOPPED = 0
    LOOPING = 1
    COUNTING = 2

    def __init__(self, file, volume = 1.0):
        self.lock = threading.Lock()
        self.state = self.STOPPED
        self.chan = None
        self.sound = mixer.Sound(file)
        self.sound.set_volume(volume)
        self.staleness = 0
        self.file = file

    def loop(self):  
        self.lock.acquire()
        self.staleness = 0
        try:
            if self.state == self.COUNTING:
                stop()
            
            if self.state == self.STOPPED:
                self.chan = self.sound.play(-1)
            
            self.state = self.LOOPING
        finally:
            self.lock.release()

    def stop(self):
        if self.state != self.STOPPED:
            self.lock.acquire()
            try:
                self.chan.fadeout(300)
                self.state = self.STOPPED
            finally:
                self.lock.release()

    def single(self):
        self.lock.acquire()
        self.staleness = 0
        try:
            if self.state == self.LOOPING:
                stop()
            
            if self.state == self.STOPPED:
                self.chan = self.sound.play()
            else: # Already counting
                self.chan.queue(self.sound) # This will only allow one extra one to be enqueued

            self.state = self.COUNTING
        finally:
            self.lock.release()

    def command(self, cmd):
         if cmd == SoundRequest.PLAY_STOP:
             self.stop()
         elif cmd == SoundRequest.PLAY_ONCE:
             self.single()
         elif cmd == SoundRequest.PLAY_START:
             self.loop()

    def get_staleness(self):
        if self.chan.get_busy():
            self.staleness = 0
        else:
            self.staleness = self.staleness + 1
        return self.staleness

class soundplay:
    def stopdict(self,dict):
        for sound in dict.values():
            sound.stop()
    
    def stopall(self):
        self.stopdict(self.builtinsounds)
        self.stopdict(self.filesounds)
        self.stopdict(self.voicesounds)

    def callback(self,data):
        self.mutex.acquire()
        
        try:
            if data.sound == SoundRequest.ALL and data.command == SoundRequest.PLAY_STOP:
                self.stopall()
            else:
                if data.sound == SoundRequest.PLAY_FILE:
                    if not data.arg in self.filesounds.keys():
                        rospy.logdebug('command for uncached wave: "%s"'%data.arg)
                        try:
                            self.filesounds[data.arg] = soundtype(data.arg)
                        except:
                            rospy.logerr('Error setting up to play "%s". Does this file exist on the machine on which sound_play is running?'%data.arg)
                            return
                    else:
                        rospy.logdebug('command for cached wave: "%s"'%data.arg)
                    sound = self.filesounds[data.arg]
                elif data.sound == SoundRequest.SAY:
                    if not data.arg in self.voicesounds.keys():
                        rospy.logdebug('command for uncached text: "%s"'%data.arg)
                        txtfilename='/tmp/play_sound_text_temp.txt'
                        wavfilename='/tmp/play_sound_wave_temp.wav'
                        f = open(txtfilename, 'w')
                        try:
                            f.write(data.arg)
                        finally:
                            f.close()
                        try:
                            os.remove(wavfilename);
                        except OSError:
                            pass
                        os.system('text2wave '+txtfilename+' -o '+wavfilename)
                        try:
                            os.stat(wavfilename)
                        except OSError:
                            rospy.logerr('Sound synthesis failed. Is festival installed? Is a festival voice installed? Try running "rosdep satisfy sound_play|sh". Refer to http://pr.willowgarage.com/wiki/sound_play/Troubleshooting')
                            return
                        self.voicesounds[data.arg] = soundtype(wavfilename)
                    else:
                        rospy.logdebug('command for cached text: "%s"'%data.arg)
                    sound = self.voicesounds[data.arg]
                else:
                    rospy.logdebug('command for builting wave: %i'%data.sound)
                    sound = self.builtinsounds[data.sound]
                sound.command(data.command)
        except Exception, e:
            rospy.logerr('Exception in callback: %s'%str(e))
            rospy.loginfo(traceback.format_exc())
        finally:
            self.mutex.release()

    # Purge sounds that haven't been played in a while.
    def cleanupdict(self, dict):
        purgelist = []
        for (key,sound) in dict.iteritems():
            try:
                staleness = sound.get_staleness()
            except Exception, e:
                rospy.logerr('Exception in cleanupdict: %s'%str(e))
                staleness = 100 # Something is wrong. Let's purge and try again.
            #print "%s %i"%(key, staleness)
            if staleness >= 10:
                purgelist.append(key)
        for key in purgelist:
           del dict[key]
    
    def cleanup(self):
        #print "cleanup %i files %i voices"%(len(self.filesounds),len(self.voicesounds))
        self.cleanupdict(self.filesounds)
        self.cleanupdict(self.voicesounds)

    def __init__(self):
        rospy.init_node('sound_play')

        rootdir = os.path.join(os.path.dirname(__file__),'..','sounds')
        
        self.mutex = threading.Lock()
        while True:
            try:
                mixer.init(11025, -16, 1, 4000)
                self.builtinsounds = {
                        SoundRequest.BACKINGUP              : soundtype(os.path.join(rootdir, 'BACKINGUP.ogg'), 0.1),
                        SoundRequest.NEEDS_UNPLUGGING       : soundtype(os.path.join(rootdir, 'NEEDS_UNPLUGGING.ogg')),
                        SoundRequest.NEEDS_PLUGGING         : soundtype(os.path.join(rootdir, 'NEEDS_PLUGGING.ogg')),
                        SoundRequest.NEEDS_UNPLUGGING_BADLY : soundtype(os.path.join(rootdir, 'NEEDS_UNPLUGGING_BADLY.ogg')),
                        SoundRequest.NEEDS_PLUGGING_BADLY   : soundtype(os.path.join(rootdir, 'NEEDS_PLUGGING_BADLY.ogg')),
                        }
                self.filesounds = {}
                self.voicesounds = {}
                self.hotlist = []
                break
            except Exception, e:
                rospy.logerr('Exception in sound startup, will retry in 5 seconds. Is the speaker connected? Have you configured ALSA? Can aplay play sound? See the wiki if there is a red light on the Logitech speaker. Have a look at http://pr.willowgarage.com/wiki/sound_play/Troubleshooting Error message: %s'%str(e))
                rospy.sleep(5);

        rospy.Subscriber("robotsound", SoundRequest, self.callback)

        rospy.loginfo('sound_play node is ready to play sound')

        while not rospy.is_shutdown():
            try:
                rospy.sleep(1)   
            except rospy.exceptions.ROSInterruptException:
                break
            self.mutex.acquire()
            try:
                self.cleanup()
            except:
                rospy.loginfo('Exception in cleanup: %s'%sys.exc_info()[0])

            self.mutex.release()

if __name__ == '__main__':
    soundplay()

