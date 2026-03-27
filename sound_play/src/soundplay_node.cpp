// **********************************************************
//  Software License Agreement (BSD License)
//
//   Copyright (c) 2009, Willow Garage, Inc.
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above
//      copyright notice, this list of conditions and the following
//      disclaimer in the documentation and/or other materials provided
//      with the distribution.
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//   POSSIBILITY OF SUCH DAMAGE.
// **********************************************************

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

#include <gst/gst.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sound_play/msg/sound_request.hpp"
#include "sound_play/action/sound_request.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

enum class SoundState
{
  STOPPED = 0,
  LOOPING = 1,
  COUNTING = 2
};

class SoundType
{
public:
  SoundType(
    rclcpp::Node * node,
    const std::string & file,
    const std::string & device,
    float volume = 1.0)
  : node_(node),
    state_(SoundState::STOPPED),
    uri_(file),
    file_(file),
    volume_(volume),
    staleness_(1)
  {
    sound_ = gst_element_factory_make("playbin", nullptr);
    if (!sound_) {
      throw std::runtime_error("Could not create sound player");
    }

    if (!device.empty() && device != "default") {
      GstElement * sink = gst_element_factory_make("alsasink", "sink");
      g_object_set(G_OBJECT(sink), "device", device.c_str(), nullptr);
      g_object_set(G_OBJECT(sound_), "audio-sink", sink, nullptr);
    }

    std::string uri;
    if (file.find(":") != std::string::npos) {
      uri = file;
    } else if (std::ifstream(file).good()) {
      uri = "file://" + (file[0] == '/' ? file : std::filesystem::absolute(file).string());
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Error: URI is invalid: %s", file.c_str());
      uri = file;
    }

    uri_ = uri;
    g_object_set(G_OBJECT(sound_), "uri", uri_.c_str(), nullptr);
    g_object_set(G_OBJECT(sound_), "volume", static_cast<double>(volume_), nullptr);

    bus_ = gst_element_get_bus(sound_);
  }

  ~SoundType()
  {
    dispose();
  }

  void update()
  {
    if (bus_) {
      gst_bus_poll(bus_, GST_MESSAGE_ERROR, 10000000);
    }
  }

  void loop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    staleness_ = 0;
    
    if (state_ == SoundState::COUNTING) {
      stop_unlocked();
    }

    if (state_ == SoundState::STOPPED) {
      gst_element_seek_simple(
        sound_, GST_FORMAT_TIME,
        GST_SEEK_FLAG_FLUSH, 0);
      gst_element_set_state(sound_, GST_STATE_PLAYING);
    }
    state_ = SoundState::LOOPING;
  }

  void dispose()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (sound_) {
      gst_element_set_state(sound_, GST_STATE_NULL);
      if (bus_) {
        gst_object_unref(GST_OBJECT(bus_));
        bus_ = nullptr;
      }
      gst_object_unref(GST_OBJECT(sound_));
      sound_ = nullptr;
      state_ = SoundState::STOPPED;
    }
  }

  void stop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_unlocked();
  }

  void single()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_DEBUG(node_->get_logger(), "Playing %s", uri_.c_str());
    staleness_ = 0;
    
    if (state_ == SoundState::LOOPING) {
      stop_unlocked();
    }

    gst_element_seek_simple(sound_, GST_FORMAT_TIME, GST_SEEK_FLAG_FLUSH, 0);
    gst_element_set_state(sound_, GST_STATE_PLAYING);
    state_ = SoundState::COUNTING;
  }

  void command(int8_t cmd)
  {
    if (cmd == sound_play::msg::SoundRequest::PLAY_STOP) {
      stop();
    } else if (cmd == sound_play::msg::SoundRequest::PLAY_ONCE) {
      single();
    } else if (cmd == sound_play::msg::SoundRequest::PLAY_START) {
      loop();
    }
  }

  int get_staleness()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gint64 position = 0;
    gint64 duration = 0;
    
    gst_element_query_position(sound_, GST_FORMAT_TIME, &position);
    gst_element_query_duration(sound_, GST_FORMAT_TIME, &duration);

    if (position != duration) {
      staleness_ = 0;
    } else {
      staleness_++;
    }
    return staleness_;
  }

  bool get_playing()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_ == SoundState::COUNTING;
  }

  float get_volume() const
  {
    return volume_;
  }

  int staleness_;

private:
  void stop_unlocked()
  {
    if (state_ != SoundState::STOPPED) {
      gst_element_set_state(sound_, GST_STATE_NULL);
      state_ = SoundState::STOPPED;
    }
  }

  rclcpp::Node * node_;
  std::mutex mutex_;
  SoundState state_;
  GstElement * sound_;
  GstBus * bus_;
  std::string uri_;
  std::string file_;
  float volume_;
};

class SoundPlayNode : public rclcpp::Node
{
public:
  using SoundRequestAction = sound_play::action::SoundRequest;
  using GoalHandleSoundRequest = rclcpp_action::ServerGoalHandle<SoundRequestAction>;

  SoundPlayNode()
  : Node("sound_play"),
    initialized_(false),
    active_sounds_(0),
    num_channels_(10)
  {
    gst_init(nullptr, nullptr);

    this->declare_parameter("loop_rate", 100);
    this->declare_parameter("device", "default");
    this->declare_parameter("default_voice", "voice_kal_diphone");

    loop_rate_ = this->get_parameter("loop_rate").as_int();
    device_ = this->get_parameter("device").as_string();
    default_voice_ = this->get_parameter("default_voice").as_string();

    diagnostic_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 1);

    std::string rootdir = ament_index_cpp::get_package_share_directory("sound_play") + "/sounds";

    builtin_sound_params_[sound_play::msg::SoundRequest::BACKINGUP] =
      std::make_pair(rootdir + "/BACKINGUP.ogg", 0.1f);
    builtin_sound_params_[sound_play::msg::SoundRequest::NEEDS_UNPLUGGING] =
      std::make_pair(rootdir + "/NEEDS_UNPLUGGING.ogg", 1.0f);
    builtin_sound_params_[sound_play::msg::SoundRequest::NEEDS_PLUGGING] =
      std::make_pair(rootdir + "/NEEDS_PLUGGING.ogg", 1.0f);
    builtin_sound_params_[sound_play::msg::SoundRequest::NEEDS_UNPLUGGING_BADLY] =
      std::make_pair(rootdir + "/NEEDS_UNPLUGGING_BADLY.ogg", 1.0f);
    builtin_sound_params_[sound_play::msg::SoundRequest::NEEDS_PLUGGING_BADLY] =
      std::make_pair(rootdir + "/NEEDS_PLUGGING_BADLY.ogg", 1.0f);

    sub_ = this->create_subscription<sound_play::msg::SoundRequest>(
      "robotsound", 10,
      std::bind(&SoundPlayNode::callback, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<SoundRequestAction>(
      this,
      "sound_play",
      std::bind(&SoundPlayNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SoundPlayNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&SoundPlayNode::handle_accepted, this, std::placeholders::_1));

    // For ros startup race condition
    std::this_thread::sleep_for(500ms);
    diagnostics(1);

    timer_ = this->create_wall_timer(
      100ms, std::bind(&SoundPlayNode::spin_once, this));

    RCLCPP_INFO(this->get_logger(), "sound_play node is ready to play sound");
    initialized_ = true;
  }

  ~SoundPlayNode()
  {
    stopall();
  }

private:
  void spin_once()
  {
    try {
      idle_loop();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in idle_loop: %s", e.what());
    }
    diagnostics(0);
  }

  void stopdict(std::map<std::string, std::shared_ptr<SoundType>> & dict)
  {
    for (auto & pair : dict) {
      pair.second->stop();
    }
  }

  void stopall()
  {
    stopdict(builtin_sounds_);
    stopdict(file_sounds_);
    stopdict(voice_sounds_);
  }

  std::shared_ptr<SoundType> select_sound(const sound_play::msg::SoundRequest & data)
  {
    if (data.sound == sound_play::msg::SoundRequest::PLAY_FILE) {
      std::string key;
      if (data.arg2.empty()) {
        key = data.arg;
      } else {
        key = ament_index_cpp::get_package_share_directory(data.arg2) + "/" + data.arg;
      }

      if (file_sounds_.find(key) == file_sounds_.end()) {
        RCLCPP_DEBUG(this->get_logger(), "command for uncached wave: %s", key.c_str());
        try {
          file_sounds_[key] = std::make_shared<SoundType>(this, key, device_, data.volume);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Error setting up to play '%s'. Does this file exist?", key.c_str());
          return nullptr;
        }
      } else {
        RCLCPP_DEBUG(this->get_logger(), "command for cached wave: %s", key.c_str());
      }
      return file_sounds_[key];

    } else if (data.sound == sound_play::msg::SoundRequest::SAY) {
      std::string voice_key = data.arg + "---" + data.arg2;
      
      if (voice_sounds_.find(voice_key) == voice_sounds_.end()) {
        RCLCPP_DEBUG(this->get_logger(), "command for uncached text: %s", voice_key.c_str());
        
        char txt_template[] = "/tmp/sound_play_XXXXXX.txt";
        char wav_template[] = "/tmp/sound_play_XXXXXX.wav";
        
        int txt_fd = mkstemps(txt_template, 4);
        int wav_fd = mkstemps(wav_template, 4);
        
        if (txt_fd == -1 || wav_fd == -1) {
          RCLCPP_ERROR(this->get_logger(), "Failed to create temporary files");
          if (txt_fd != -1) {close(txt_fd);}
          if (wav_fd != -1) {close(wav_fd);}
          return nullptr;
        }
        
        std::string txt_filename(txt_template);
        std::string wav_filename(wav_template);
        
        close(wav_fd);
        
        // Write text to file
        write(txt_fd, data.arg.c_str(), data.arg.length());
        close(txt_fd);
        
        std::string voice = data.arg2.empty() ? default_voice_ : data.arg2;
        
        // Call text2wave
        std::string cmd = "text2wave -eval '(" + voice + ")' " + 
                         txt_filename + " -o " + wav_filename;
        int ret = system(cmd.c_str());
        
        if (ret != 0 || std::ifstream(wav_filename).peek() == std::ifstream::traits_type::eof()) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Sound synthesis failed. Is festival installed?");
          unlink(txt_filename.c_str());
          unlink(wav_filename.c_str());
          return nullptr;
        }
        
        unlink(txt_filename.c_str());
        
        try {
          voice_sounds_[voice_key] = std::make_shared<SoundType>(
            this, wav_filename, device_, data.volume);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(this->get_logger(), "Error creating voice sound: %s", e.what());
          unlink(wav_filename.c_str());
          return nullptr;
        }
      } else {
        RCLCPP_DEBUG(this->get_logger(), "command for cached text: %s", voice_key.c_str());
      }
      return voice_sounds_[voice_key];

    } else {
      // Builtin sound
      RCLCPP_DEBUG(this->get_logger(), "command for builtin wave: %d", data.sound);
      
      if (builtin_sounds_.find(std::to_string(data.sound)) == builtin_sounds_.end() ||
        builtin_sounds_[std::to_string(data.sound)]->get_volume() != data.volume)
      {
        if (builtin_sound_params_.find(data.sound) == builtin_sound_params_.end()) {
          RCLCPP_ERROR(this->get_logger(), "Unknown builtin sound: %d", data.sound);
          return nullptr;
        }
        
        auto params = builtin_sound_params_[data.sound];
        float volume = data.volume;
        if (params.second != 1.0f) {
          volume = (volume + params.second) / 2.0f;
        }
        
        builtin_sounds_[std::to_string(data.sound)] =
          std::make_shared<SoundType>(this, params.first, device_, volume);
      }
      return builtin_sounds_[std::to_string(data.sound)];
    }
  }

  void callback(const sound_play::msg::SoundRequest::SharedPtr data)
  {
    if (!initialized_) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    try {
      if (data->sound == sound_play::msg::SoundRequest::ALL &&
        data->command == sound_play::msg::SoundRequest::PLAY_STOP)
      {
        stopall();
      } else {
        auto sound = select_sound(*data);
        if (sound) {
          sound->command(data->command);
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in callback: %s", e.what());
    }
  }

  void cleanupdict(std::map<std::string, std::shared_ptr<SoundType>> & dict)
  {
    std::vector<std::string> purgelist;
    
    for (auto & pair : dict) {
      try {
        int staleness = pair.second->get_staleness();
        if (staleness >= 10) {
          purgelist.push_back(pair.first);
        }
        if (staleness == 0) {
          active_sounds_++;
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Exception in cleanupdict for sound (%s): %s",
          pair.first.c_str(), e.what());
        purgelist.push_back(pair.first);
      }
    }
    
    for (const auto & key : purgelist) {
      RCLCPP_DEBUG(this->get_logger(), "Purging %s from cache", key.c_str());
      dict[key]->dispose();
      dict.erase(key);
    }
  }

  void cleanup()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      active_sounds_ = 0;
      cleanupdict(file_sounds_);
      cleanupdict(voice_sounds_);
      cleanupdict(builtin_sounds_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in cleanup: %s", e.what());
    }
  }

  void diagnostics(int state)
  {
    try {
      auto da = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
      diagnostic_msgs::msg::DiagnosticStatus ds;
      
      ds.name = std::string(this->get_name()) + ": Node State";
      
      if (state == 0) {
        ds.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        ds.message = std::to_string(active_sounds_) + " sounds playing";
        
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "Active sounds";
        kv.value = std::to_string(active_sounds_);
        ds.values.push_back(kv);
        
        kv.key = "Allocated sound channels";
        kv.value = std::to_string(num_channels_);
        ds.values.push_back(kv);
        
        kv.key = "Buffered builtin sounds";
        kv.value = std::to_string(builtin_sounds_.size());
        ds.values.push_back(kv);
        
        kv.key = "Buffered wave sounds";
        kv.value = std::to_string(file_sounds_.size());
        ds.values.push_back(kv);
        
        kv.key = "Buffered voice sounds";
        kv.value = std::to_string(voice_sounds_.size());
        ds.values.push_back(kv);
      } else if (state == 1) {
        ds.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        ds.message = "Sound device not open yet.";
      } else {
        ds.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        ds.message = "Can't open sound device.";
      }
      
      da->status.push_back(ds);
      da->header.stamp = this->now();
      diagnostic_pub_->publish(std::move(da));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in diagnostics: %s", e.what());
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const SoundRequestAction::Goal> /*goal*/)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleSoundRequest> /*goal_handle*/)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSoundRequest> goal_handle)
  {
    std::thread{std::bind(&SoundPlayNode::execute, this, std::placeholders::_1), goal_handle}
    .detach();
  }

  void execute(const std::shared_ptr<GoalHandleSoundRequest> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<SoundRequestAction::Result>();
    auto feedback = std::make_shared<SoundRequestAction::Feedback>();

    if (!initialized_) {
      RCLCPP_ERROR(this->get_logger(), "soundplay_node is not initialized yet.");
      goal_handle->abort(result);
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    stopall();

    try {
      const auto & data = goal->sound_request;
      
      if (data.sound == sound_play::msg::SoundRequest::ALL &&
        data.command == sound_play::msg::SoundRequest::PLAY_STOP)
      {
        stopall();
      } else {
        auto sound = select_sound(data);
        if (!sound) {
          goal_handle->abort(result);
          return;
        }
        
        sound->command(data.command);

        auto start_time = this->now();
        bool success = true;

        while (sound->get_playing()) {
          if (goal_handle->is_canceling()) {
            RCLCPP_INFO(this->get_logger(), "sound_play action: Canceled");
            sound->stop();
            goal_handle->canceled(result);
            return;
          }

          sound->update();
          feedback->playing = sound->get_playing();
          feedback->stamp = (this->now() - start_time);
          goal_handle->publish_feedback(feedback);

          std::this_thread::sleep_for(
            std::chrono::milliseconds(1000 / loop_rate_));
        }

        if (success) {
          result->playing = feedback->playing;
          result->stamp = feedback->stamp;
          RCLCPP_INFO(this->get_logger(), "sound_play action: Succeeded");
          goal_handle->succeed(result);
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in action callback: %s", e.what());
      goal_handle->abort(result);
    }
  }

  void idle_loop()
  {
    cleanup();
  }

  rclcpp::Subscription<sound_play::msg::SoundRequest>::SharedPtr sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp_action::Server<SoundRequestAction>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  bool initialized_;
  int active_sounds_;
  int num_channels_;
  int loop_rate_;
  std::string device_;
  std::string default_voice_;

  std::map<int8_t, std::pair<std::string, float>> builtin_sound_params_;
  std::map<std::string, std::shared_ptr<SoundType>> builtin_sounds_;
  std::map<std::string, std::shared_ptr<SoundType>> file_sounds_;
  std::map<std::string, std::shared_ptr<SoundType>> voice_sounds_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SoundPlayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
