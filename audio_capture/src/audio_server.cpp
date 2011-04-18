#include <stdio.h>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <boost/thread.hpp>

#include <ros/ros.h>

#include "audio_msgs/AudioData.h"

namespace audio_transport
{
  class RosGstServer
  {
    public: 
      RosGstServer()
      {
        std::string source_type;

        _sleep_time = 0;
        _bitrate = 192;

        // The bitrate at which to encode the audio
        ros::param::param<int>("~bitrate", _bitrate, 192);

        // The source of the audio
        ros::param::param<std::string>("~src", source_type, "alsasrc");

        _pub = _nh.advertise<audio_msgs::AudioData>("audio", 10, true);

        _loop = g_main_loop_new(NULL, false);
        _pipeline = gst_pipeline_new("ros_pipeline");

        // We create the sink first, just for convenience
        _sink = gst_element_factory_make("appsink", "sink");
        g_object_set(G_OBJECT(_sink), "emit-signals", true, NULL);
        g_object_set(G_OBJECT(_sink), "max-buffers", 100, NULL);
        g_signal_connect( G_OBJECT(_sink), "new-buffer", 
                          G_CALLBACK(onNewBuffer), this);

        if (source_type == "alsasrc")
        {
          _source = gst_element_factory_make("alsasrc", "source");
          _convert = gst_element_factory_make("audioconvert", "convert");

          _encode = gst_element_factory_make("lame", "encoder");
          g_object_set( G_OBJECT(_encode), "preset", 1001, NULL);
          g_object_set( G_OBJECT(_encode), "bitrate", _bitrate, NULL);
          
          gst_bin_add_many( GST_BIN(_pipeline), _source, _convert, _encode, _sink, NULL);
          gst_element_link_many(_source, _convert, _encode, _sink, NULL);
        }
        else
        {
          _sleep_time = 10000;
          _source = gst_element_factory_make("filesrc", "source");
          g_object_set(G_OBJECT(_source), "location", source_type.c_str(), NULL);

          gst_bin_add_many( GST_BIN(_pipeline), _source, _sink, NULL);
          gst_element_link_many(_source, _sink, NULL);
        }

        gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);

        _gst_thread = boost::thread::thread( boost::bind(g_main_loop_run, _loop) );
      }

      void publish( const audio_msgs::AudioData &msg )
      {
        _pub.publish(msg);

        if (_sleep_time > 0)
          usleep(_sleep_time);
      }

      static GstFlowReturn onNewBuffer (GstAppSink *appsink, gpointer userData)
      {
        RosGstServer *server = reinterpret_cast<RosGstServer*>(userData);

        GstBuffer *buffer;
        g_signal_emit_by_name(appsink, "pull-buffer", &buffer);

        audio_msgs::AudioData msg;
        msg.data.resize( buffer->size );
        memcpy( &msg.data[0], buffer->data, buffer->size);

        server->publish(msg);

        return GST_FLOW_OK;
      }

    private:
      ros::NodeHandle _nh;
      ros::Publisher _pub;

      boost::thread _gst_thread;

      GstElement *_pipeline, *_source, *_sink, *_convert, *_encode;
      GMainLoop *_loop;
      int _sleep_time;
      int _bitrate;
  };
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "audio_server");
  gst_init(&argc, &argv);

  audio_transport::RosGstServer server;
  ros::spin();
}
