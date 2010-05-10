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

import roslib; roslib.load_manifest('sound_play')

import rospy
import threading
from sound_play.msg import SoundRequest
import os
import logging
import sys
import traceback
import tempfile
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue, DiagnosticArray

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
        self.lock = threading.RLock()
        self.state = self.STOPPED
        self.chan = None
        self.sound = mixer.Sound(file)
        self.sound.set_volume(volume)
        self.staleness = 1
        self.file = file

    def loop(self):  
        #print "loop"
        #print "lock"
        self.lock.acquire()
        try:
            self.staleness = 0
            if self.state == self.COUNTING:
                self.stop()
            
            if self.state == self.STOPPED:
                self.chan = self.sound.play(-1)
            
            self.state = self.LOOPING
        finally:
            self.lock.release()
            #print "unlock done"
        #print "loop done"

    def stop(self):
        #print "stop"
        if self.state != self.STOPPED:
            #print "lock"
            self.lock.acquire()
            #print "lock done"
            try:
                #print "fadeout"
                self.chan.fadeout(300)
                #print "fadeout done"
                self.state = self.STOPPED
            finally:
                self.lock.release()
                #print "unlock done"
        #print "stop done"

    def single(self):
        #print "single"
        #print "lock"
        self.lock.acquire()
        try:
            self.staleness = 0
            if self.state == self.LOOPING:
                self.stop()
            
            if self.state == self.STOPPED:
                self.chan = self.sound.play()
            else: # Already counting
                self.chan.queue(self.sound) # This will only allow one extra one to be enqueued

            self.state = self.COUNTING
        finally:
            self.lock.release()
            #print "unlock done"
        #print "single done"

    def command(self, cmd):
         if cmd == SoundRequest.PLAY_STOP:
             self.stop()
         elif cmd == SoundRequest.PLAY_ONCE:
             self.single()
         elif cmd == SoundRequest.PLAY_START:
             self.loop()

    def get_staleness(self):
        #print "lock"
        self.lock.acquire()
        try:
            if self.chan.get_busy():
                self.staleness = 0
            else:
                self.staleness = self.staleness + 1
            return self.staleness
        finally:
            self.lock.release()
            #print "unlock done"

class soundplay:
    def stopdict(self,dict):
        for sound in dict.values():
            sound.stop()
    
    def stopall(self):
        self.stopdict(self.builtinsounds)
        self.stopdict(self.filesounds)
        self.stopdict(self.voicesounds)

    def callback(self,data):
        #print "callback", str(data)
        if not self.initialized:
            return
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
                        txtfile = tempfile.NamedTemporaryFile(prefix='sound_play', suffix='txt')
                        wavfile = tempfile.NamedTemporaryFile(prefix='sound_play', suffix='wav')
                        txtfilename=txtfile.name
                        wavfilename=wavfile.name
                        try:
                            txtfile.write(data.arg)
                            txtfile.flush()
                            os.system('text2wave '+txtfilename+' -o '+wavfilename)
                            try:
                                if os.stat(wavfilename).st_size == 0:
                                    raise OSError # So we hit the same catch block
                            except OSError:
                                rospy.logerr('Sound synthesis failed. Is festival installed? Is a festival voice installed? Try running "rosdep satisfy sound_play|sh". Refer to http://pr.willowgarage.com/wiki/sound_play/Troubleshooting')
                                return
                            self.voicesounds[data.arg] = soundtype(wavfilename)
                        finally:
                            txtfile.close()
                            wavfile.close()
                    else:
                        rospy.logdebug('command for cached text: "%s"'%data.arg)
                    sound = self.voicesounds[data.arg]
                else:
                    rospy.logdebug('command for builtin wave: %i'%data.sound)
                    if not data.sound in self.builtinsounds:
                        params = self.builtinsoundparams[data.sound]
                        self.builtinsounds[data.sound] = soundtype(params[0], params[1])
                    sound = self.builtinsounds[data.sound]
                if sound.staleness != 0 and data.command != SoundRequest.PLAY_STOP:
                    # This sound isn't counted in active_sounds
                    #print "activating %i %s"%(data.sound,data.arg)
                    self.active_sounds = self.active_sounds + 1
                    sound.staleness = 0
#                    if self.active_sounds > self.num_channels:
#                        mixer.set_num_channels(self.active_sounds)
#                        self.num_channels = self.active_sounds
                sound.command(data.command)
        except Exception, e:
            rospy.logerr('Exception in callback: %s'%str(e))
            rospy.loginfo(traceback.format_exc())
        finally:
            self.mutex.release()
            #print "done callback"

    # Purge sounds that haven't been played in a while.
    def cleanupdict(self, dict):
        purgelist = []
        for (key,sound) in dict.iteritems():
            try:
                staleness = sound.get_staleness()
            except Exception, e:
                rospy.logerr('Exception in cleanupdict for sound (%s): %s'%(str(key),str(e)))
                staleness = 100 # Something is wrong. Let's purge and try again.
            #print "%s %i"%(key, staleness)
            if staleness >= 10:
                purgelist.append(key)
            if staleness == 0: # Sound is playing
                self.active_sounds = self.active_sounds + 1
        for key in purgelist:
           del dict[key]
    
    def cleanup(self):
        #print "cleanup %i files %i voices"%(len(self.filesounds),len(self.voicesounds))
        self.mutex.acquire()
        try:
            self.active_sounds = 0
            self.cleanupdict(self.filesounds)
            self.cleanupdict(self.voicesounds)
            self.cleanupdict(self.builtinsounds)
        except:
            rospy.loginfo('Exception in cleanup: %s'%sys.exc_info()[0])
        finally:
            self.mutex.release()

    def diagnostics(self, state):
        try:
            da = DiagnosticArray()
            ds = DiagnosticStatus()
            ds.name = rospy.get_caller_id().lstrip('/') + ": Node State"
            if state == 0:
                ds.level = DiagnosticStatus.OK
                ds.message = "%i sounds playing"%self.active_sounds
                ds.values.append(KeyValue("Active sounds", str(self.active_sounds)))
                ds.values.append(KeyValue("Allocated sound channels", str(self.num_channels)))
                ds.values.append(KeyValue("Buffered builtin sounds", str(len(self.builtinsounds))))
                ds.values.append(KeyValue("Buffered wave sounds", str(len(self.filesounds))))
                ds.values.append(KeyValue("Buffered voice sounds", str(len(self.voicesounds))))
            elif state == 1:
                ds.level = DiagnosticStatus.WARN
                ds.message = "Sound device not open yet."
            else:
                ds.level = DiagnosticStatus.ERROR
                ds.message = "Can't open sound device. See http://pr.willowgarage.com/wiki/sound_play/Troubleshooting"
            da.status.append(ds)
            da.header.stamp = rospy.get_rostime()
            self.diagnostic_pub.publish(da)
        except Exception, e:
            rospy.loginfo('Exception in diagnostics: %s'%str(e))

    def __init__(self):
        rospy.init_node('sound_play')
        self.diagnostic_pub = rospy.Publisher("/diagnostics", DiagnosticArray)

        rootdir = os.path.join(os.path.dirname(__file__),'..','sounds')
        
        self.builtinsoundparams = {
                SoundRequest.BACKINGUP              : (os.path.join(rootdir, 'BACKINGUP.ogg'), 0.1),
                SoundRequest.NEEDS_UNPLUGGING       : (os.path.join(rootdir, 'NEEDS_UNPLUGGING.ogg'), 1),
                SoundRequest.NEEDS_PLUGGING         : (os.path.join(rootdir, 'NEEDS_PLUGGING.ogg'), 1),
                SoundRequest.NEEDS_UNPLUGGING_BADLY : (os.path.join(rootdir, 'NEEDS_UNPLUGGING_BADLY.ogg'), 1),
                SoundRequest.NEEDS_PLUGGING_BADLY   : (os.path.join(rootdir, 'NEEDS_PLUGGING_BADLY.ogg'), 1),
                }
        
        self.mutex = threading.Lock()
        sub = rospy.Subscriber("robotsound", SoundRequest, self.callback)
        self.mutex.acquire()
        self.no_error = True
        self.initialized = False
        self.active_sounds = 0
        self.sleep(0.5) # For ros startup race condition
        self.diagnostics(1)
        while not rospy.is_shutdown():
            while not rospy.is_shutdown() and self.mixer_init():
                self.no_error = True
                self.initialized = True
                self.mutex.release()
                try:
                    self.idle_loop()
                    # Returns after inactive period to test device availability
                    #print "Exiting idle"
                except:
                    rospy.loginfo('Exception in idle_loop: %s'%sys.exc_info()[0])
                finally:
                    self.mutex.acquire()
                    mixer.quit()
            self.diagnostics(2)
        self.mutex.release()

    def mixer_init(self): 
        try:
            mixer.init(11025, -16, 1, 4000)
            self.init_vars()
            return True
        except Exception, e:
            if self.no_error:
                rospy.logerr('Exception in sound startup, will retry once per second. Is the speaker connected? Have you configured ALSA? Can aplay play sound? See the wiki if there is a red light on the Logitech speaker. Have a look at http://pr.willowgarage.com/wiki/sound_play/Troubleshooting Error message: %s'%str(e))
                self.no_error = False
                self.initialized = False
            self.sleep(1);
        return False

    def init_vars(self):
        self.num_channels = 10
        mixer.set_num_channels(self.num_channels)
        self.builtinsounds = {}
        self.filesounds = {}
        self.voicesounds = {}
        self.hotlist = []
        if not self.initialized:
            rospy.loginfo('sound_play node is ready to play sound')
            
    def sleep(self, duration):
        try:    
            rospy.sleep(duration)   
        except rospy.exceptions.ROSInterruptException:
            pass
    
    def idle_loop(self):
        self.last_activity_time = rospy.get_time()
        while (rospy.get_time() - self.last_activity_time < 10 or
                 len(self.builtinsounds) + len(self.voicesounds) + len(self.filesounds) > 0) \
                and not rospy.is_shutdown():
            #print "idle_loop"
            self.diagnostics(0)
            self.sleep(1)
            self.cleanup()
        #print "idle_exiting"

if __name__ == '__main__':
    soundplay()

