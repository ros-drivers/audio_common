# ROS audio\_common Package
[![](https://github.com/ros-drivers/audio_common/actions/workflows/main.yml/badge.svg?=master)](https://github.com/ros-drivers/audio_common/actions/workflows/main.yml)

This repository contains the ROS audio\_common package.

For user documentation, please refer to the [ROS Wiki page for audio\_common](http://wiki.ros.org/audio_common)

# Deb Build Status

| Package              | Melodic (Bionic)                                                                                                                                                                                     | Noetic (Focal)                                                                                                                                                                                     | Noetic (Buster)                                                                                                                                                                                      |
|:---------------------|:-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|:---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|:-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| audio_common (arm64) | [![Build Status](http://build.ros.org/job/Mbin_ubv8_uBv8__audio_common__ubuntu_bionic_arm64__binary/badge/icon)](http://build.ros.org/job/Mbin_ubv8_uBv8__audio_common__ubuntu_bionic_arm64__binary) | [![Build Status](http://build.ros.org/job/Nbin_ufv8_uFv8__audio_common__ubuntu_focal_arm64__binary/badge/icon)](http://build.ros.org/job/Nbin_ufv8_uFv8__audio_common__ubuntu_focal_arm64__binary) | [![Build Status](http://build.ros.org/job/Nbin_dbv8_dBv8__audio_common__debian_buster_arm64__binary/badge/icon)](http://build.ros.org/job/Nbin_dbv8_dBv8__audio_common__debian_buster_arm64__binary) |
| audio_common (armhf) | [![Build Status](http://build.ros.org/job/Mbin_ubhf_uBhf__audio_common__ubuntu_bionic_armhf__binary/badge/icon)](http://build.ros.org/job/Mbin_ubhf_uBhf__audio_common__ubuntu_bionic_armhf__binary) | [![Build Status](http://build.ros.org/job/Nbin_ufhf_uFhf__audio_common__ubuntu_focal_armhf__binary/badge/icon)](http://build.ros.org/job/Nbin_ufhf_uFhf__audio_common__ubuntu_focal_armhf__binary) | ---                                                                                                                                                                                                  |
| audio_common (i386)  | ---                                                                                                                                                                                                  | ---                                                                                                                                                                                                | ---                                                                                                                                                                                                  |
| audio_common (amd64) | [![Build Status](http://build.ros.org/job/Mbin_uB64__audio_common__ubuntu_bionic_amd64__binary/badge/icon)](http://build.ros.org/job/Mbin_uB64__audio_common__ubuntu_bionic_amd64__binary)           | [![Build Status](http://build.ros.org/job/Nbin_uF64__audio_common__ubuntu_focal_amd64__binary/badge/icon)](http://build.ros.org/job/Nbin_uF64__audio_common__ubuntu_focal_amd64__binary)           | [![Build Status](http://build.ros.org/job/Nbin_db_dB64__audio_common__debian_buster_amd64__binary/badge/icon)](http://build.ros.org/job/Nbin_db_dB64__audio_common__debian_buster_amd64__binary)     |

# Support

Please ask support questions on [ROS Answers](http://answers.ros.org/questions/).

# Building from source

On ROS Indigo or Jade, the `indigo-devel` branch is recommended.

On ROS Kinetic, Melodic and Noetic, the `master` branch is recommended.

# Development, Branch and Release Policy

This package is not under active development, but is accepting pull requests for bug fixes and new features. (Development may be done for serious bug fixes; pending maintainer time).

The `sound_play`, `groovy-devel` and `hydro-devel` branches are from previous ROS releases and are frozen; no new pull requests will be accepted on these branches.

The `indigo-devel` branch is the stable branch; only bug fixes are accepted on this branch. Periodic releases are done from `indigo-devel` into ROS Indigo and ROS Jade, with version numbers in the 0.2.x range.

The `master` branch is currently considered the development branch, and is released into ROS Kinetic, Melodic and Noetic with version numbers in the 0.3.x range. `master` is accepting new, non-breaking features and bug fixes.

Large, breaking changes such as changes to dependencies or the package API will be considered, but they will probably be staged into a development branch for release into the next major release of ROS (ROS L)
