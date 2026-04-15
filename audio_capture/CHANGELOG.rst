^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package audio_capture
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2026-04-14)
------------------
* Merge pull request `#274 <https://github.com/ros-drivers/audio_common/issues/274>`_ from knorth55/pass-audio-convert
  pass to audio convert after _source
* pass audio convert
* Merge pull request `#272 <https://github.com/ros-drivers/audio_common/issues/272>`_ from sbgisen/bugfix/pulsesrc-pipeline
  Fix crash when using wave format with pulsesrc
* Add audioconvert
* Merge pull request `#270 <https://github.com/ros-drivers/audio_common/issues/270>`_ from socialminds-ai/audio-capture-qol
  Minor quality-of-life fixes for audio_capture
* Merge pull request `#269 <https://github.com/ros-drivers/audio_common/issues/269>`_ from socialminds-ai/capture-src
  audio_capture: make it possible to configure the gstreamer input plugin
* audio_capture: document the node parameters
* audio_capture: print out errors in the gstreamer pipeline
* audio_capture: make it possible to configure the gstreamer input plugin
  This is especially useful to use pulseaudio (pulsesrc) instead of
  alsa, which is convenient when working inside Docker containers
* Merge pull request `#267 <https://github.com/ros-drivers/audio_common/issues/267>`_ from knorth55/ros2-fix-rolling
  replace ament_target_dependencies
* remove CMAKE_CXX flags
* replace ament_target_dependencies
* Merge pull request `#257 <https://github.com/ros-drivers/audio_common/issues/257>`_ from sbgisen/feature/qos
  Set transient local to audio info publisher
* Merge remote-tracking branch 'origin/ros2' into feature/qos
* Merge pull request `#254 <https://github.com/ros-drivers/audio_common/issues/254>`_ from sbgisen/feature/diagnostic
  Publish diagnostic
* Set transient local to audio info publisher
* Auto detect rate
* Publish diagnostic
* Merge pull request `#236 <https://github.com/ros-drivers/audio_common/issues/236>`_ from anrp-tri/anrp/info
  Publish audio_info on a schedule
* Merge pull request `#237 <https://github.com/ros-drivers/audio_common/issues/237>`_ from anrp-tri/anrp/flexsrc
* Allow audio_capture node to specify src type
  Allows use of pulsesrc and other options.
* Publish audio_info on a schedule
  Make sure that latecomers can get the codec details.
* Merge pull request `#224 <https://github.com/ros-drivers/audio_common/issues/224>`_ from knorth55/ros2-backport
* on real systems publish system clock time in capture node
* publish audio stamped in audio_capture.cpp
* The capture node is hard-coded to alsasrc
* Merge pull request `#215 <https://github.com/ros-drivers/audio_common/issues/215>`_ from knorth55/weeshal/components
* use include for audio_capture launch.xml
* fix typo in capture_to_file.launch.py
* add missing build_depend
* add launch.pya in audio_capture
* rename to audio_capture_node
* converted nodes to components
* Merge pull request `#208 <https://github.com/ros-drivers/audio_common/issues/208>`_ from weeshal/bugfix/parameter_names
* removing ros1 namespacing
* Merge pull request `#188 <https://github.com/ros-drivers/audio_common/issues/188>`_ from Patrick-AA/ros2-patch
* changed ns to push-ros-namespace
* Merge pull request `#178 <https://github.com/ros-drivers/audio_common/issues/178>`_ from knorth55/ros2
* audio_capture: migrate to ros2
* Contributors: Andrew Patrikalakis, Patrick-AA, Shingo Kitagawa, Séverin Lemaignan, Tatsuro Sakaguchi, Vishal Giridhar, v4hn, vishalgi

0.3.12 (2021-09-01)
-------------------
* Merge branch 'master' into master
* Contributors: Shingo Kitagawa

0.3.11 (2021-04-08)
-------------------

0.3.10 (2021-01-07)
-------------------
* add bitrate in capture launch
* [audio_capture] Publish audio info once before publishing /audio
* Contributors: Naoya Yamaguchi, Shingo Kitagawa

0.3.9 (2020-10-22)
------------------
* Merge pull request `#160 <https://github.com/ros-drivers/audio_common/issues/160>`_ from knorth55/add-device-play
* use ROS_INFO instead of printf
* Contributors: Shingo Kitagawa

0.3.8 (2020-09-13)
------------------

0.3.7 (2020-08-08)
------------------
* Merge pull request `#150 <https://github.com/ros-drivers/audio_common/issues/150>`_ from sktometometo/fix_mp3_options
  Fix property of lamemp3enc element in audio_capture so that the bitrate parameter work properly.
* fix property of lamemp3enc element so that it will use the specified bitrate
* Merge pull request `#146 <https://github.com/ros-drivers/audio_common/issues/146>`_ from knorth55/mp3-support
* use space instead of tab
* use same caps
* support channls for mp3
* Merge pull request `#145 <https://github.com/ros-drivers/audio_common/issues/145>`_ from knorth55/mp3-channel-rate
  [audio_capture] add sample_format in audio_capture
* Merge pull request `#147 <https://github.com/ros-drivers/audio_common/issues/147>`_ from knorth55/fix-filesink
  [audio_capture] fix filesink for wave format
* add sample_format arg in capture_to_file.launch
* fix filesink for wave format
* add sample_format in audio_capture
* Contributors: Koki Shinjo, Shingo Kitagawa

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
* Merge branch 'master' of github.com:ros-drivers/audio_common
* Contributors: Gerard Canal

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
* Merge pull request `#102 <https://github.com/ros-drivers/audio_common/issues/102>`_ from EndPointCorp/fix_capture_leak
  Fix audio_capture leak
* Fix audio_capture sample/buffer leak
* Merge pull request `#90 <https://github.com/ros-drivers/audio_common/issues/90>`_ from prarobo/master
  Error checking code and improvements to launch files
* Bug fix
* fix(audio_capture): capturing wave using gst1.0
  0.10-style raw audio caps were being created, according to GStreamer warning. Should be audio/x-raw,format=(string).. now.
* Merge pull request `#1 <https://github.com/ros-drivers/audio_common/issues/1>`_ from prarobo/fixes
  Error checking code and improvements to launch files
* Removed default device
* Added error checking code
* Added parameters to launch files
* Contributors: Austin, Matt Vollrath, Prasanna Kannappan, Rokus, Yuki Furuta, prarobo

0.3.1 (2016-08-28)
------------------
* Update to new gstreamer rosdeps
* #70 can launch these in different namespaces with different microphones, and both are operating.
* #70 can switch between different microphones, but the first microphone doesn't like the hw:1, it only works with device:="" - so must be doing something wrong still.
* Add changelogs
* [audio_capture] add error handler
* [audio_capture] add option to publish captured audio data as wav format
  Conflicts:
  audio_capture/src/audio_capture.cpp
* Fixed memory leak (see #18).
* Removed trailing whitespace.
* Fixed problem that CMake uses gstreamer-0.1 instead of gstreamer-1.0
* Added gstreamer 1.0 dependecies
* Ported to gstreamer 1.0
  package.xml dependencies still missing
* Update maintainer email
* Contributors: Benny, Felix Duvallet, Furushchev, Lucas Walter, trainman419

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
* [audio_capture] add error handler
* [audio_capture] add option to publish captured audio data as wav format
* Fixed memory leak (see `#18 <https://github.com/ros-drivers/audio_common/issues/18>`_).
* Removed trailing whitespace.
* Contributors: Felix Duvallet, Furushchev, trainman419

0.2.8 (2015-10-02)
------------------
* Update maintainer email
* Contributors: trainman419

0.2.7 (2014-07-25)
------------------
* audio_capture.cpp has to wait for generated AudioData headers
* Contributors: v4hn

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
* Update CMakeLists.txt
* audio_capture: install launchfiles
* Contributors: David Gossow

0.2.3 (2013-07-15)
------------------
* Fix install rule for audio_capture.
* Contributors: Austin Hendrix

0.2.2 (2013-04-10)
------------------

0.2.1 (2013-04-08 13:59)
------------------------

0.2.0 (2013-04-08 13:49)
------------------------
* Finish catkinizing audio_common.
* Catkinize audio_play.
* Catkinize audio_capture.
* Fix typo in package.xml
* Versions and more URLs.
* Convert manifests to package.xml
* Convert audio_capture manifest to package.xml
* Ditch old makefiles.
* Updates manifest
* Updated manifests for rodep2
* oneiric build fixes, bump version to 0.1.6
* Removed redundant thread::thread
* Added a rosdep.yaml file
* Fixed to use audio_common_msgs
* Added ability to use different festival voices
* Updated documentation
* Added ability to capture to file
* Fixed ignore files
* Added hgignore files
* Audio_capture and audio_play working
* Making separate audio_capture and audio_play packages
* Moved audio_transport to audio_capture
* Contributors: Austin Hendrix, Brian Gerkey, Nate Koenig, nkoenig
