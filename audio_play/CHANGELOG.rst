^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package audio_play
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.11 (2021-04-08)
-------------------

0.3.10 (2021-01-07)
-------------------

0.3.9 (2020-10-22)
------------------
* Merge pull request `#160 <https://github.com/ros-drivers/audio_common/issues/160>`_ from knorth55/add-device-play
* refactor audio_play to use same code
* add audioresample in audio_play
* apply caps for both formats
* add device for wave format
* add sync false for alsasink
* use alsasink
* add depth rosparam
* add device arg in play.launch
* fix audio_play to save file
* Contributors: Shingo Kitagawa

0.3.8 (2020-09-13)
------------------
* Merge pull request `#151 <https://github.com/ros-drivers/audio_common/issues/151>`_ from knorth55/do-timestamp-false
  [audio_play] set do_timestamp false
* set do_timestamp false
* Contributors: Shingo Kitagawa

0.3.7 (2020-08-08)
------------------
* Merge pull request `#146 <https://github.com/ros-drivers/audio_common/issues/146>`_ from knorth55/mp3-support
* support format, rate, channels in mp3
* Merge pull request `#127 <https://github.com/ros-drivers/audio_common/issues/127>`_ from knorth55/audio-play-wave
  [audio_play] support wave format
* add sample_format param in audio_play
* add channels and sample_rate in audio_play/play.launch
* add channels and sample_rate in audio_play.cpp
* add format arg in play.launch
* support wave in audio_play
* Merge pull request `#144 <https://github.com/ros-drivers/audio_common/issues/144>`_ from ros-drivers/knorth55-patch-1
* add gstreamer1.0-alsa for run_depend in audio_play
* Contributors: Shingo Kitagawa

0.3.6 (2020-05-29)
------------------
* Merge pull request `#141 <https://github.com/ros-drivers/audio_common/issues/141>`_ from knorth55/add-maintainer
  add maintainer
* add maintainer
* Contributors: Shingo Kitagawa

0.3.5 (2020-04-28)
------------------

0.3.4 (2020-04-02)
------------------
* audio_play fix for reproducing livestream sound (`#122 <https://github.com/ros-drivers/audio_common/issues/122>`_)
  * Added capability to read from a udpsrc. To generalize it, possibly similarly to gscam.
  * Added parameter to control do-timestamp, as this fixes the problem with audio_play not being able to play livestream sound.
  * Aligning with the base master, removing the changes from the branch that included the udpsrc.
  Co-authored-by: Alberto Quattrini Li <albertoq@cse.sc.edu>
* Merge branch 'master' of github.com:ros-drivers/audio_common
* Contributors: Alberto Quattrini Li, Gerard Canal

0.3.3 (2018-05-22)
------------------

0.3.2 (2018-05-02)
------------------
* [sound_play] add option to select audio device to play / record (`#87 <https://github.com/ros-drivers/audio_common/issues/87>`_)
  * [sound_play] add option to select audio device to play
  * [sound_play] reformat README to markdown; add usage to set device via rosparam
  * audio_capture: add option for selecting device to use
  * audio_play: add option to select device for playing audio
  * add device argument to launch files
  Conflicts:
  audio_capture/launch/capture.launch
  audio_capture/launch/capture_to_file.launch
  audio_capture/src/audio_capture.cpp
  audio_play/launch/play.launch
  sound_play/scripts/soundplay_node.py
* Merge pull request `#101 <https://github.com/ros-drivers/audio_common/issues/101>`_ from EndPointCorp/audio_play_dont_pause_pipeline
  audio_play: Fix mp3 clip overlap by never pausing the pipeline
* audio_play: Don't pause the pipeline
  This prevents glitches when playing short mp3 clips.
* Merge pull request `#90 <https://github.com/ros-drivers/audio_common/issues/90>`_ from prarobo/master
  Error checking code and improvements to launch files
* Merge pull request `#1 <https://github.com/ros-drivers/audio_common/issues/1>`_ from prarobo/fixes
  Error checking code and improvements to launch files
* Added parameters to launch files
* Contributors: Austin, Matt Vollrath, Prasanna Kannappan, Yuki Furuta, prarobo

0.3.1 (2016-08-28)
------------------
* Update to new gstreamer rosdeps
* #70 can launch these in different namespaces with different microphones, and both are operating.
* Add changelogs
* Changed message level to warning
* Fixed problem that CMake uses gstreamer-0.1 instead of gstreamer-1.0
* Fixed underflow.
  Before the sink buffer underflows the pipeline is paused. When data is received again the pipeline is set to playing again.
* Added gstreamer 1.0 dependecies
* Ported to gstreamer 1.0
  package.xml dependencies still missing
* Change audio sink to autoaudiosink
* Update maintainer email
* Contributors: Benny, Hans Gaiser, Lucas Walter, trainman419

0.2.11 (2016-02-16)
-------------------
* Add changelogs
* Contributors: trainman419

0.2.10 (2016-01-21)
-------------------
* Add changelogs
* Contributors: trainman419

0.2.9 (2015-12-02)
------------------
* Add changelogs
* Contributors: trainman419

0.2.8 (2015-10-02)
------------------
* Changed message level to warning
* Fixed underflow.
  Before the sink buffer underflows the pipeline is paused. When data is received again the pipeline is set to playing again.
* Change audio sink to autoaudiosink
* Update maintainer email
* Contributors: Benny, Hans Gaiser, trainman419

0.2.7 (2014-07-25)
------------------

0.2.6 (2014-02-26)
------------------
* audio_capture and play _require\_ gstreamer, it's not optional
* Contributors: v4hn

0.2.5 (2014-01-23)
------------------
* "0.2.5"
* Contributors: trainman419

0.2.4 (2013-09-10)
------------------

0.2.3 (2013-07-15)
------------------
* Fix dependencies and install rules.
* Contributors: Austin Hendrix

0.2.2 (2013-04-10)
------------------

0.2.1 (2013-04-08 13:59)
------------------------

0.2.0 (2013-04-08 13:49)
------------------------
* Finish catkinizing audio_common.
* Catkinize audio_play.
* Fix typo in package.xml
* Versions and more URLs.
* Convert manifests to package.xml
* Ditch old makefiles.
* Updates manifest
* Updated manifests for rodep2
* oneiric build fixes, bump version to 0.1.6
* Removed another duplicate thread::thread
* Added a rosdep.yaml file
* Fixed to use audio_common_msgs
* Added ability to use different festival voices
* Updated documentation
* Update to audio_play
* Fixed ignore files
* Added hgignore files
* Audio_capture and audio_play working
* Making separate audio_capture and audio_play packages
* Contributors: Austin Hendrix, Brian Gerkey, Nate Koenig, nkoenig
