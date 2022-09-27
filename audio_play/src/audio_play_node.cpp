#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <boost/thread.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "audio_common_msgs/msg/audio_data.hpp"

namespace audio_play
{
  class AudioPlayNode: public rclcpp::Node
  {
    public:
      AudioPlayNode(const rclcpp::NodeOptions &options)
      : Node("audio_play_node", options)
      {
        gst_init(nullptr, nullptr);

        GstPad *audiopad;
        GstCaps *caps;

        std::string dst_type;
        std::string device;
        bool do_timestamp;
        std::string format;
        int channels;
        int depth;
        int sample_rate;
        std::string sample_format;

        // The destination of the audio
        this->declare_parameter<std::string>("dst", "alsasink");
        this->declare_parameter<std::string>("device", std::string());
        this->declare_parameter<bool>("do_timestamp",  true);
        this->declare_parameter<std::string>("format", "mp3");
        this->declare_parameter<int>("channels", 1);
        this->declare_parameter<int>("depth", 16);
        this->declare_parameter<int>("sample_rate", 16000);
        this->declare_parameter<std::string>("sample_format", "S16LE");

        this->get_parameter("dst", dst_type);
        this->get_parameter("device", device);
        this->get_parameter("do_timestamp", do_timestamp);
        this->get_parameter("format", format);
        this->get_parameter("channels", channels);
        this->get_parameter("depth", depth);
        this->get_parameter("sample_rate", sample_rate);
        this->get_parameter("sample_format", sample_format);

        _sub = this->create_subscription<audio_common_msgs::msg::AudioData>(
            "audio", 10, std::bind(&AudioPlayNode::onAudio, this, std::placeholders::_1));

        _loop = g_main_loop_new(NULL, false);

        _pipeline = gst_pipeline_new("app_pipeline");
        _source = gst_element_factory_make("appsrc", "app_source");
        g_object_set(G_OBJECT(_source), "do-timestamp", (do_timestamp) ? TRUE : FALSE, NULL);

        //_playbin = gst_element_factory_make("playbin2", "uri_play");
        //g_object_set( G_OBJECT(_playbin), "uri", "file:///home/test/test.mp3", NULL);
        caps = gst_caps_new_simple(
            "audio/x-raw",
            "format", G_TYPE_STRING, sample_format.c_str(),
            "rate", G_TYPE_INT, sample_rate,
            "channels", G_TYPE_INT, channels,
            "width",    G_TYPE_INT, depth,
            "depth",    G_TYPE_INT, depth,
            "signed",   G_TYPE_BOOLEAN, TRUE,
            "layout", G_TYPE_STRING, "interleaved",
            NULL);

        if (dst_type == "alsasink")
        {
          _audio = gst_bin_new("audiobin");
          _convert = gst_element_factory_make("audioconvert", "convert");
          audiopad = gst_element_get_static_pad(_convert, "sink");
          _resample = gst_element_factory_make("audioresample", "resample");

          _sink = gst_element_factory_make("alsasink", "sink");
          g_object_set(G_OBJECT(_sink), "sync", FALSE, NULL);
          if (!device.empty()) {
            g_object_set(G_OBJECT(_sink), "device", device.c_str(), NULL);
          }
          gst_bin_add_many( GST_BIN(_audio), _convert, _resample, _sink, NULL);
          gst_element_link_many(_convert, _resample, _sink, NULL);
          gst_element_add_pad(_audio, gst_ghost_pad_new("sink", audiopad));
        }
        else
        {
          RCLCPP_INFO_STREAM(this->get_logger(), "file sink to " << dst_type.c_str());
          _sink = gst_element_factory_make("filesink", "sink");
          g_object_set(G_OBJECT(_sink), "location", dst_type.c_str(), NULL);
        }

        if (format == "mp3")
        {
          if (dst_type == "alsasink")
          {
            _decoder = gst_element_factory_make("decodebin", "decoder");
            g_signal_connect(_decoder, "pad-added", G_CALLBACK(cb_newpad),this);

            _filter = gst_element_factory_make("capsfilter", "filter");
            g_object_set( G_OBJECT(_filter), "caps", caps, NULL);

            gst_bin_add_many(GST_BIN(_pipeline), _source, _decoder, _filter, _audio, NULL);
            gst_element_link_many(_source, _decoder, _filter, _audio, NULL);
            gst_object_unref(audiopad);
            gst_caps_unref(caps);
          }
          else
          {
            gst_bin_add_many(GST_BIN(_pipeline), _source, _sink, NULL);
            gst_element_link(_source, _sink);
          }
        }
        else if (format == "wave")
        {
          g_object_set( G_OBJECT(_source), "caps", caps, NULL);
          g_object_set (G_OBJECT (_source), "format", GST_FORMAT_TIME, NULL);
          if (dst_type == "alsasink")
          {
            gst_bin_add_many( GST_BIN(_pipeline), _source, _audio, NULL);
            gst_element_link_many( _source, _audio, NULL);
            gst_object_unref(audiopad);
          }
          else
          {
            _filter = gst_element_factory_make("wavenc", "filter");
            gst_bin_add_many(GST_BIN(_pipeline), _source, _filter, _sink, NULL);
            gst_element_link_many( _source, _filter, _sink, NULL);
          }
          gst_caps_unref(caps);
        }
        else
        {
          RCLCPP_ERROR_STREAM(this->get_logger(), "Unsupported format: " << format.c_str());
        }

        gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);
        //gst_element_set_state(GST_ELEMENT(_playbin), GST_STATE_PLAYING);

        _gst_thread = boost::thread( boost::bind(g_main_loop_run, _loop) );
      }

    private:

      void onAudio(const audio_common_msgs::msg::AudioData::SharedPtr msg) const
      {
        GstBuffer *buffer = gst_buffer_new_and_alloc(msg->data.size());
        gst_buffer_fill(buffer, 0, &msg->data[0], msg->data.size());
        GstFlowReturn ret;

        g_signal_emit_by_name(_source, "push-buffer", buffer, &ret);
        gst_buffer_unref(buffer);
      }

     static void cb_newpad (GstElement *decodebin, GstPad *pad, 
                             gpointer data)
      {
        AudioPlayNode *client = reinterpret_cast<AudioPlayNode*>(data);

        GstCaps *caps;
        GstStructure *str;
        GstPad *audiopad;

        /* only link once */
        audiopad = gst_element_get_static_pad (client->_audio, "sink");
        if (GST_PAD_IS_LINKED (audiopad)) 
        {
          g_object_unref (audiopad);
          return;
        }

        /* check media type */
        caps = gst_pad_query_caps (pad, NULL);
        str = gst_caps_get_structure (caps, 0);
        if (!g_strrstr (gst_structure_get_name (str), "audio")) {
          gst_caps_unref (caps);
          gst_object_unref (audiopad);
          return;
        }

        gst_caps_unref (caps);

        /* link'n'play */
        gst_pad_link (pad, audiopad);

        g_object_unref (audiopad);
      }

      rclcpp::Subscription<audio_common_msgs::msg::AudioData>::SharedPtr _sub;
      boost::thread _gst_thread;

      GstElement *_pipeline, *_source, *_sink, *_decoder, *_convert, *_audio, *_resample, *_filter;
      GstElement *_playbin;
      GMainLoop *_loop;
  };
}

RCLCPP_COMPONENTS_REGISTER_NODE(audio_play::AudioPlayNode)



