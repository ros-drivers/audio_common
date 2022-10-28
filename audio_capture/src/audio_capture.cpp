#include <stdio.h>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <boost/thread.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "audio_common_msgs/msg/audio_data.hpp"
#include "audio_common_msgs/msg/audio_info.hpp"

namespace audio_transport
{
  class RosGstCapture: public rclcpp::Node
  {
    public:
      RosGstCapture(const rclcpp::NodeOptions &options)
      :
       Node("audio_capture", options)
      {
        gst_init(nullptr, nullptr);

        _bitrate = 192;
        std::string dst_type;
        std::string device;

        // Need to encoding or publish raw wave data
        this->declare_parameter<std::string>("format", "mp3");
        this->declare_parameter<std::string>("sample_format", "S16LE");
        this->get_parameter("format", _format);
        this->get_parameter("sample_format", _sample_format);

        // The bitrate at which to encode the audio
        this->declare_parameter<int>("bitrate", 192);
        this->get_parameter("bitrate", _bitrate);

        // only available for raw data
        this->declare_parameter<int>("channels", 1);
        this->declare_parameter<int>("depth", 16);
        this->declare_parameter<int>("sample_rate", 16000);
        this->get_parameter("channels", _channels);
        this->get_parameter("depth", _depth);
        this->get_parameter("sample_rate", _sample_rate);

        // The destination of the audio
        this->declare_parameter<std::string>("dst", "appsink");
        this->get_parameter("dst", dst_type);

        // The source of the audio
        //this->declare_parameter<std::string>("src", source_type, "alsasrc");
        this->declare_parameter<std::string>("device", "");
        this->get_parameter("device", device);

        _pub = this->create_publisher<audio_common_msgs::msg::AudioData>("audio", 10);
        _pub_info = this->create_publisher<audio_common_msgs::msg::AudioInfo>("audio_info", 1);

        _loop = g_main_loop_new(NULL, false);
        _pipeline = gst_pipeline_new("ros_pipeline");
        _bus = gst_pipeline_get_bus(GST_PIPELINE(_pipeline));
        gst_bus_add_signal_watch(_bus);
        g_signal_connect(_bus, "message::error",
                         G_CALLBACK(onMessage), this);
        g_object_unref(_bus);

        // We create the sink first, just for convenience
        if (dst_type == "appsink")
        {
          _sink = gst_element_factory_make("appsink", "sink");
          g_object_set(G_OBJECT(_sink), "emit-signals", true, NULL);
          g_object_set(G_OBJECT(_sink), "max-buffers", 100, NULL);
          g_signal_connect( G_OBJECT(_sink), "new-sample",
                            G_CALLBACK(onNewBuffer), this);
        }
        else
        {
          RCLCPP_INFO_STREAM(this->get_logger(), "file sink to " << dst_type.c_str());
          _sink = gst_element_factory_make("filesink", "sink");
          g_object_set( G_OBJECT(_sink), "location", dst_type.c_str(), NULL);
        }

        _source = gst_element_factory_make("alsasrc", "source");
        // if device isn't specified, it will use the default which is
        // the alsa default source.
        // A valid device will be of the foram hw:0,0 with other numbers
        // than 0 and 0 as are available.
        if (device != "")
        {
          // ghcar *gst_device = device.c_str();
          g_object_set(G_OBJECT(_source), "device", device.c_str(), NULL);
        }

        GstCaps *caps;
        caps = gst_caps_new_simple("audio/x-raw",
                                   "format", G_TYPE_STRING, _sample_format.c_str(),
                                   "channels", G_TYPE_INT, _channels,
                                   "width",    G_TYPE_INT, _depth,
                                   "depth",    G_TYPE_INT, _depth,
                                   "rate",     G_TYPE_INT, _sample_rate,
                                   "signed",   G_TYPE_BOOLEAN, TRUE,
                                   NULL);

        gboolean link_ok;
        if (_format == "mp3"){
          _filter = gst_element_factory_make("capsfilter", "filter");
          g_object_set( G_OBJECT(_filter), "caps", caps, NULL);
          gst_caps_unref(caps);

          _convert = gst_element_factory_make("audioconvert", "convert");
          if (!_convert) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to create audioconvert element");
            exitOnMainThread(1);
          }

          _encode = gst_element_factory_make("lamemp3enc", "encoder");
          if (!_encode) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to create encoder element");
            exitOnMainThread(1);
          }
          g_object_set( G_OBJECT(_encode), "target", 1, NULL);
          g_object_set( G_OBJECT(_encode), "bitrate", _bitrate, NULL);

          gst_bin_add_many( GST_BIN(_pipeline), _source, _filter, _convert, _encode, _sink, NULL);
          link_ok = gst_element_link_many(_source, _filter, _convert, _encode, _sink, NULL);
        } else if (_format == "wave") {
          if (dst_type == "appsink") {
            g_object_set( G_OBJECT(_sink), "caps", caps, NULL);
            gst_caps_unref(caps);
            gst_bin_add_many( GST_BIN(_pipeline), _source, _sink, NULL);
            link_ok = gst_element_link_many( _source, _sink, NULL);
          } else {
            _filter = gst_element_factory_make("wavenc", "filter");
            gst_bin_add_many( GST_BIN(_pipeline), _source, _filter, _sink, NULL);
            link_ok = gst_element_link_many( _source, _filter, _sink, NULL);
          }
        } else {
          RCLCPP_ERROR_STREAM(this->get_logger(), "format must be \"wave\" or \"mp3\"");
          exitOnMainThread(1);
        }
        /*}
        else
        {
          _sleep_time = 10000;
          _source = gst_element_factory_make("filesrc", "source");
          g_object_set(G_OBJECT(_source), "location", source_type.c_str(), NULL);

          gst_bin_add_many( GST_BIN(_pipeline), _source, _sink, NULL);
          gst_element_link_many(_source, _sink, NULL);
        }
        */

        if (!link_ok) {
          RCLCPP_ERROR_STREAM(this->get_logger(), "Unsupported media type.");
          exitOnMainThread(1);
        }

        gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);

        _gst_thread = boost::thread( boost::bind(g_main_loop_run, _loop) );

        audio_common_msgs::msg::AudioInfo info_msg;
        info_msg.channels = _channels;
        info_msg.sample_rate = _sample_rate;
        info_msg.sample_format = _sample_format;
        info_msg.bitrate = _bitrate;
        info_msg.coding_format = _format;
        _pub_info->publish(info_msg);
      }

      ~RosGstCapture()
      {
        g_main_loop_quit(_loop);
        gst_element_set_state(_pipeline, GST_STATE_NULL);
        gst_object_unref(_pipeline);
        g_main_loop_unref(_loop);
      }

      void exitOnMainThread(int code)
      {
        exit(code);
      }

      void publish( const audio_common_msgs::msg::AudioData &msg )
      {
        _pub->publish(msg);
      }

      static GstFlowReturn onNewBuffer (GstAppSink *appsink, gpointer userData)
      {
        RosGstCapture *server = reinterpret_cast<RosGstCapture*>(userData);
        GstMapInfo map;

        GstSample *sample;
        g_signal_emit_by_name(appsink, "pull-sample", &sample);

        GstBuffer *buffer = gst_sample_get_buffer(sample);

        audio_common_msgs::msg::AudioData msg;
        gst_buffer_map(buffer, &map, GST_MAP_READ);
        msg.data.resize( map.size );

        memcpy( &msg.data[0], map.data, map.size );

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);

        server->publish(msg);

        return GST_FLOW_OK;
      }

      static gboolean onMessage (GstBus *bus, GstMessage *message, gpointer userData)
      {
        RosGstCapture *server = reinterpret_cast<RosGstCapture*>(userData);
        GError *err;
        gchar *debug;

        gst_message_parse_error(message, &err, &debug);
        // RCLCPP_ERROR_STREAM(this->get_logger(), "gstreamer: " << err->message);
        g_error_free(err);
        g_free(debug);
        g_main_loop_quit(server->_loop);
        server->exitOnMainThread(1);
        return FALSE;
      }

    private:
      rclcpp::Publisher<audio_common_msgs::msg::AudioData>::SharedPtr _pub;
      rclcpp::Publisher<audio_common_msgs::msg::AudioInfo>::SharedPtr _pub_info;

      boost::thread _gst_thread;

      GstElement *_pipeline, *_source, *_filter, *_sink, *_convert, *_encode;
      GstBus *_bus;
      int _bitrate, _channels, _depth, _sample_rate;
      GMainLoop *_loop;
      std::string _format, _sample_format;
  };
}

RCLCPP_COMPONENTS_REGISTER_NODE(audio_transport::RosGstCapture)
