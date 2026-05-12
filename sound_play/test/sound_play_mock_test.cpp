/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <sound_play/sound_play.hpp>
#include <sound_play_msgs/msg/sound_request.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>

using SoundRequest = sound_play_msgs::msg::SoundRequest;

class TestSoundPlayClient : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    client_node_ = std::make_shared<rclcpp::Node>("sound_client_node", "/test");
    listener_node_ = std::make_shared<rclcpp::Node>("sound_listener_node", "/test");

    sound_client_ = std::make_unique<sound_play::SoundClient>(client_node_);
    sound_client_->setQuiet(true);

    last_msg_.reset();
    sub_ = listener_node_->create_subscription<SoundRequest>(
      "robotsound", rclcpp::QoS(10),
      [this](const SoundRequest::SharedPtr msg) {last_msg_ = msg;});

    executor_.add_node(client_node_);
    executor_.add_node(listener_node_);
    spinFor(std::chrono::milliseconds(50));
  }

  void TearDown()
  {
    executor_.remove_node(client_node_);
    executor_.remove_node(listener_node_);
    sound_client_.reset();
    sub_.reset();
    client_node_.reset();
    listener_node_.reset();
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some(std::chrono::milliseconds(10));
    }
  }

  SoundRequest::SharedPtr waitForMessage(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(500))
  {
    last_msg_.reset();
    auto end = std::chrono::steady_clock::now() + timeout;
    while (!last_msg_ && std::chrono::steady_clock::now() < end) {
      executor_.spin_some(std::chrono::milliseconds(10));
    }
    return last_msg_;
  }
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Node::SharedPtr listener_node_;
  std::unique_ptr<sound_play::SoundClient> sound_client_;
  rclcpp::Subscription<SoundRequest>::SharedPtr sub_;
  SoundRequest::SharedPtr last_msg_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

// --- say / repeat / stopSaying ---

TEST_F(TestSoundPlayClient, say_publishes_correct_message) {
  sound_client_->say("hello world");
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->sound, SoundRequest::SAY);
  EXPECT_EQ(msg->command, SoundRequest::PLAY_ONCE);
  EXPECT_EQ(msg->arg, "hello world");
  EXPECT_FLOAT_EQ(msg->volume, 1.0f);
}

TEST_F(TestSoundPlayClient, say_with_custom_volume) {
  sound_client_->say("test", "voice_kal_diphone", 0.5f);
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_FLOAT_EQ(msg->volume, 0.5f);
}

TEST_F(TestSoundPlayClient, repeat_publishes_play_start) {
  sound_client_->repeat("loop this");
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->sound, SoundRequest::SAY);
  EXPECT_EQ(msg->command, SoundRequest::PLAY_START);
  EXPECT_EQ(msg->arg, "loop this");
}

TEST_F(TestSoundPlayClient, stop_saying_publishes_play_stop) {
  sound_client_->stopSaying("hello world");
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->sound, SoundRequest::SAY);
  EXPECT_EQ(msg->command, SoundRequest::PLAY_STOP);
  EXPECT_EQ(msg->arg, "hello world");
}

// --- playWave / startWave / stopWave ---

TEST_F(TestSoundPlayClient, play_wave_publishes_correct_message) {
  sound_client_->playWave("/abs/path/test.wav");
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->sound, SoundRequest::PLAY_FILE);
  EXPECT_EQ(msg->command, SoundRequest::PLAY_ONCE);
  EXPECT_EQ(msg->arg, "/abs/path/test.wav");
}

TEST_F(TestSoundPlayClient, start_wave_publishes_play_start) {
  sound_client_->startWave("/abs/path/test.wav");
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->sound, SoundRequest::PLAY_FILE);
  EXPECT_EQ(msg->command, SoundRequest::PLAY_START);
}

TEST_F(TestSoundPlayClient, stop_wave_publishes_play_stop) {
  sound_client_->stopWave("/abs/path/test.wav");
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->sound, SoundRequest::PLAY_FILE);
  EXPECT_EQ(msg->command, SoundRequest::PLAY_STOP);
  EXPECT_EQ(msg->arg, "/abs/path/test.wav");
}

// --- playWaveFromPkg / startWaveFromPkg / stopWaveFromPkg ---

TEST_F(TestSoundPlayClient, play_wave_from_pkg_publishes_correct_message) {
  sound_client_->playWaveFromPkg("sound_play", "say-beep", 0.5f);
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->sound, SoundRequest::PLAY_FILE);
  EXPECT_EQ(msg->command, SoundRequest::PLAY_ONCE);
  EXPECT_EQ(msg->arg, "say-beep");
  EXPECT_EQ(msg->arg2, "sound_play");
  EXPECT_FLOAT_EQ(msg->volume, 0.5f);
}

TEST_F(TestSoundPlayClient, start_wave_from_pkg_publishes_play_start) {
  sound_client_->startWaveFromPkg("sound_play", "say-beep");
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->sound, SoundRequest::PLAY_FILE);
  EXPECT_EQ(msg->command, SoundRequest::PLAY_START);
  EXPECT_EQ(msg->arg, "say-beep");
  EXPECT_EQ(msg->arg2, "sound_play");
}

TEST_F(TestSoundPlayClient, stop_wave_from_pkg_publishes_play_stop) {
  sound_client_->stopWaveFromPkg("sound_play", "say-beep");
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->sound, SoundRequest::PLAY_FILE);
  EXPECT_EQ(msg->command, SoundRequest::PLAY_STOP);
  EXPECT_EQ(msg->arg, "say-beep");
  EXPECT_EQ(msg->arg2, "sound_play");
}

// --- built-in sounds: play / start / stop / stopAll ---

TEST_F(TestSoundPlayClient, play_builtin_publishes_correct_message) {
  sound_client_->play(SoundRequest::BACKINGUP);
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->sound, SoundRequest::BACKINGUP);
  EXPECT_EQ(msg->command, SoundRequest::PLAY_ONCE);
}

TEST_F(TestSoundPlayClient, start_builtin_publishes_play_start) {
  sound_client_->start(SoundRequest::NEEDS_PLUGGING);
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->sound, SoundRequest::NEEDS_PLUGGING);
  EXPECT_EQ(msg->command, SoundRequest::PLAY_START);
}

TEST_F(TestSoundPlayClient, stop_builtin_publishes_play_stop) {
  sound_client_->stop(SoundRequest::NEEDS_PLUGGING);
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->sound, SoundRequest::NEEDS_PLUGGING);
  EXPECT_EQ(msg->command, SoundRequest::PLAY_STOP);
}

TEST_F(TestSoundPlayClient, stop_all_sends_all_stop) {
  sound_client_->stopAll();
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->sound, SoundRequest::ALL);
  EXPECT_EQ(msg->command, SoundRequest::PLAY_STOP);
}

// --- volume clamping ---

TEST_F(TestSoundPlayClient, volume_clamped_above_one) {
  sound_client_->say("test", "voice_kal_diphone", 2.0f);
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_FLOAT_EQ(msg->volume, 1.0f);
}

TEST_F(TestSoundPlayClient, volume_clamped_below_zero) {
  sound_client_->say("test", "voice_kal_diphone", -0.5f);
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_FLOAT_EQ(msg->volume, 0.0f);
}

// --- Sound objects (voiceSound / waveSound / waveSoundFromPkg / builtinSound) ---

TEST_F(TestSoundPlayClient, sound_object_voice_play) {
  auto sound = sound_client_->voiceSound("hello");
  sound.play();
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->sound, SoundRequest::SAY);
  EXPECT_EQ(msg->command, SoundRequest::PLAY_ONCE);
  EXPECT_EQ(msg->arg, "hello");
}

TEST_F(TestSoundPlayClient, sound_object_voice_repeat) {
  auto sound = sound_client_->voiceSound("hello");
  sound.repeat();
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->sound, SoundRequest::SAY);
  EXPECT_EQ(msg->command, SoundRequest::PLAY_START);
}

TEST_F(TestSoundPlayClient, sound_object_voice_stop) {
  auto sound = sound_client_->voiceSound("hello");
  sound.stop();
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->sound, SoundRequest::SAY);
  EXPECT_EQ(msg->command, SoundRequest::PLAY_STOP);
}

TEST_F(TestSoundPlayClient, sound_object_wave_play) {
  auto sound = sound_client_->waveSound("/test.wav");
  sound.play();
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->sound, SoundRequest::PLAY_FILE);
  EXPECT_EQ(msg->command, SoundRequest::PLAY_ONCE);
  EXPECT_EQ(msg->arg, "/test.wav");
}

TEST_F(TestSoundPlayClient, sound_object_wave_from_pkg_play) {
  auto sound = sound_client_->waveSoundFromPkg("sound_play", "say-beep", 0.7f);
  sound.play();
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->sound, SoundRequest::PLAY_FILE);
  EXPECT_EQ(msg->command, SoundRequest::PLAY_ONCE);
  EXPECT_EQ(msg->arg, "say-beep");
  EXPECT_EQ(msg->arg2, "sound_play");
  EXPECT_FLOAT_EQ(msg->volume, 0.7f);
}

TEST_F(TestSoundPlayClient, sound_object_builtin_play) {
  auto sound = sound_client_->builtinSound(SoundRequest::BACKINGUP);
  sound.play();
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->sound, SoundRequest::BACKINGUP);
  EXPECT_EQ(msg->command, SoundRequest::PLAY_ONCE);
}

TEST_F(TestSoundPlayClient, sound_object_volume_preserved) {
  auto sound = sound_client_->voiceSound("hello", 0.6f);
  sound.play();
  auto msg = waitForMessage();
  ASSERT_NE(msg, nullptr);
  EXPECT_FLOAT_EQ(msg->volume, 0.6f);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
