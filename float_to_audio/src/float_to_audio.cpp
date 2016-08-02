#include <stdio.h>
#include <gst/gst.h>
#include <gst/audio/audio-format.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <boost/thread.hpp>

#include <ros/ros.h>

#include <audio_common_msgs/AudioData.h>
#include <sensor_msgs/ChannelFloat32.h>
// rostopic pub /samples sensor_msgs/ChannelFloat32 "{name: '', values: [-0.4, 0.0]}"
// GST_DEBUG="*:5" rosrun float_to_audio float_to_audio

// gst-launch-1.0 audiotestsrc ! audio/x-raw, format="F32LE", rate=16000 ! audioconvert ! audioresample ! audio/x-raw, format="S16LE", rate=8000 ! autoaudiosink


namespace audio_transport
{
  class RosFloatToGst
  {
    public:
      RosFloatToGst()
      {
        // Need to encoding or publish raw wave data
        ros::param::param<std::string>("~format", _format, "mp3");
        // The bitrate at which to encode the audio
        ros::param::param<int>("~bitrate", _bitrate, 192);
        // only available for raw data
        ros::param::param<int>("~channels", _channels, 1);
        ros::param::param<int>("~depth", _depth, 16);
        ros::param::param<int>("~sample_rate", _sample_rate, 16000);

        int input_sample_rate;
        ros::param::param<int>("~input_sample_rate", input_sample_rate, 16000);

        _pub = _nh.advertise<audio_common_msgs::AudioData>("audio", 10, true);

        _loop = g_main_loop_new(NULL, false);
        _pipeline = gst_pipeline_new("ros_pipeline");
        _bus = gst_pipeline_get_bus(GST_PIPELINE(_pipeline));
        gst_bus_add_signal_watch(_bus);
        g_signal_connect(_bus, "message::error",
                         G_CALLBACK(onMessage), this);
        g_object_unref(_bus);

        // We create the sink first, just for convenience
        _sink = gst_element_factory_make("appsink", "sink");
        if (_sink == NULL)
        {
          ROS_ERROR_STREAM("couldn't create sink");
          exitOnMainThread(1);
        }
        g_object_set(G_OBJECT(_sink), "emit-signals", true, NULL);
        g_object_set(G_OBJECT(_sink), "max-buffers", 100, NULL);
        g_signal_connect( G_OBJECT(_sink), "new-sample",
                          G_CALLBACK(onNewBuffer), this);

        _audioresample = gst_element_factory_make("audioresample", "audioresample");
        if (_audioresample == NULL)
        {
          ROS_ERROR_STREAM("couldn't create audioresample");
          exitOnMainThread(1);
        }

        _audioconvert = gst_element_factory_make("audioconvert", "audioconvert");
        if (_audioconvert == NULL)
        {
          ROS_ERROR_STREAM("couldn't create audioconvert");
          exitOnMainThread(1);
        }

        _source = gst_element_factory_make("appsrc", "source");
        if (_source == NULL)
        {
          ROS_ERROR_STREAM("couldn't create source");
          exitOnMainThread(1);
        }
				g_signal_connect(_source, "need-data", G_CALLBACK(cb_need_data),this);

        {
          // 'caps' -> capabilities
          GstCaps *caps;
          caps = gst_caps_new_simple("audio/x-raw",
                              "format", G_TYPE_STRING, "F32LE",
                              // "format", G_TYPE_STRING, "S16LE",
                              "channels", G_TYPE_INT, 1,
                              "layout", G_TYPE_STRING, "interleaved",  // GST_AUDIO_LAYOUT_INTERLEAVED,
                              "channel-mask", GST_TYPE_BITMASK, 0x0000000000000001,
                              // "width - bits per sample
                              // depth - bits ACTUALLY USED FOR AUDIO per sample
                              // You can have 32-bit samples, but in each
                              // 32-bit group only 16 or 24 bits
                              // will be used.
                              // This is to achieve necessary alignment."
                              "width",    G_TYPE_INT, 32,
                              "depth",    G_TYPE_INT, 32,
                              //"width",    G_TYPE_INT, _depth,
                              //"depth",    G_TYPE_INT, _depth,
                              "endianness",    G_TYPE_INT, G_BYTE_ORDER,  // 1234?
                              "rate",     G_TYPE_INT, input_sample_rate,
                              "signed",   G_TYPE_BOOLEAN, TRUE,
                              NULL);
          g_object_set( G_OBJECT(_source), "caps", caps, NULL);
          gst_caps_unref(caps);
        }

        #if 0
        // any caps set on filter enforces those as limitations on the stream
        _filter = gst_element_factory_make("capsfilter", "filter");
        if (_filter == NULL)
        {
          ROS_ERROR_STREAM("couldn't create filter");
          exitOnMainThread(1);
        }
        if (false)
        {
          GstCaps *caps;
          caps = gst_caps_new_simple("audio/x-raw",
                                     // "format", G_TYPE_STRING, "S16LE"
                                     // "channels", G_TYPE_INT, _channels,
                                     // "layout", G_TYPE_INT, GST_AUDIO_LAYOUT_INTERLEAVED,
                                     // "width",    G_TYPE_INT, _depth,
                                     // "depth",    G_TYPE_INT, _depth,
                                     "rate",     G_TYPE_INT, _sample_rate,
                                     // "signed",   G_TYPE_BOOLEAN, TRUE,
                              NULL);
          g_object_set( G_OBJECT(_filter), "caps", caps, NULL);
          gst_caps_unref(caps);
        }
        #endif

        // mp3 
        // audio/x-raw, format=(string)S16LE, layout=(string)interleaved,
        // rate=(int){ 8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000 },
        // channels=(int)1

        gboolean link_ok;
        if (_format == "mp3"){
          _encode = gst_element_factory_make("lamemp3enc", "encoder");
          if (_encode == NULL)
          {
            ROS_ERROR_STREAM("couldn't create encode");
            exitOnMainThread(1);
          }
          g_object_set( G_OBJECT(_encode), "quality", 2.0, NULL);
          g_object_set( G_OBJECT(_encode), "bitrate", _bitrate, NULL);

          link_ok = addAllToPipeline();
        }
        else if (_format == "flac")
        {
          _encode = gst_element_factory_make("flacenc", "encoder");
          link_ok = addAllToPipeline();
        #if 0
        } else if (_format == "wave") {
          // TODO(lwalter) this isn't working
          GstCaps *caps;
          caps = gst_caps_new_simple("audio/x-raw",
																		 "format", G_TYPE_STRING, "S16LE",
                                     "channels", G_TYPE_INT, _channels,
                                     // // "layout", G_TYPE_INT, GST_AUDIO_LAYOUT_INTERLEAVED,
                                     "width",    G_TYPE_INT, _depth,
                                     "depth",    G_TYPE_INT, _depth,
                                     "rate",     G_TYPE_INT, _sample_rate,
                                     "signed",   G_TYPE_BOOLEAN, TRUE,
                                     NULL);

          g_object_set( G_OBJECT(_sink), "caps", caps, NULL);
          gst_caps_unref(caps);

          gst_bin_add_many(GST_BIN(_pipeline), _source, _audioconvert,
                           _audioresample, _sink, NULL);
          link_ok = gst_element_link_many(_source, _audioconvert,
                                          _audioresample, _sink, NULL);
        #endif
        } else {
          ROS_ERROR_STREAM("format must be \"flac\" or \"mp3\"");
          exitOnMainThread(1);
        }

        if (!link_ok) {
          ROS_ERROR_STREAM("Unsupported media type.");
          exitOnMainThread(1);
        }

        gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);

        _sub = _nh.subscribe("samples", 10, &RosFloatToGst::onFloat, this);
        _gst_thread = boost::thread( boost::bind(g_main_loop_run, _loop) );
				_paused = false;
      }

      ~RosFloatToGst()
      {
        g_main_loop_quit(_loop);
        gst_element_set_state(_pipeline, GST_STATE_NULL);
        gst_object_unref(_pipeline);
        g_main_loop_unref(_loop);
      }

      #if 0
      void spin()
      {
        while (ros::ok())
        {
          ros::spinOnce();
        }
        ROS_WARN_STREAM("quitting");
        g_main_loop_quit(_loop);
        gst_element_set_state(_pipeline, GST_STATE_NULL);
        gst_object_unref(_pipeline);
        g_main_loop_unref(_loop);
      }
      #endif

      void exitOnMainThread(int code)
      {
        ros::shutdown();
        exit(code);
      }

      gboolean addAllToPipeline()
      {
        if (!gst_bin_add( GST_BIN(_pipeline), _source))
        {
          ROS_ERROR_STREAM("source");
          return false;
        }
        if (!gst_bin_add( GST_BIN(_pipeline), _audioconvert))
        {
          ROS_ERROR_STREAM("audioconvert");
          return false;
        }
        if (!gst_bin_add( GST_BIN(_pipeline), _audioresample))
        {
          ROS_ERROR_STREAM("audioresample");
          return false;
        }
        if (!gst_bin_add( GST_BIN(_pipeline), _encode))
        {
          ROS_ERROR_STREAM("encode");
          return false;
        }
        if (!gst_bin_add( GST_BIN(_pipeline), _sink))
        {
          ROS_ERROR_STREAM("sink");
          return false;
        }
        gboolean link_ok = gst_element_link_many(_source, _audioconvert, _audioresample,
            _encode, _sink, NULL);
        return link_ok;
      }

      void onFloat(const sensor_msgs::ChannelFloat32ConstPtr &msg)
      {
        if (_paused)
        {
          gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);
          _paused = false;
        }

        GstBuffer *buffer = gst_buffer_new_and_alloc(msg->values.size() * 4);
        int num_bytes = gst_buffer_fill(buffer, 0, &msg->values[0], msg->values.size() * 4);
        GstFlowReturn ret;
        g_signal_emit_by_name(_source, "push-buffer", buffer, &ret);
        ROS_DEBUG_STREAM("emitted push " << num_bytes << " " << ret);
      }

      static GstFlowReturn onNewBuffer (GstAppSink *appsink, gpointer userData)
      {
        ROS_DEBUG_STREAM("new buffer");
        RosFloatToGst *server = reinterpret_cast<RosFloatToGst*>(userData);
        GstMapInfo map;

        GstSample *sample;
        g_signal_emit_by_name(appsink, "pull-sample", &sample);

        GstBuffer *buffer = gst_sample_get_buffer(sample);

        if (buffer == NULL)
        {
          ROS_WARN_STREAM("buffer is null");
          return GST_FLOW_ERROR;
        }
        audio_common_msgs::AudioData msg;
        gst_buffer_map(buffer, &map, GST_MAP_READ);
        msg.data.resize( map.size );

        memcpy( &msg.data[0], map.data, map.size );

        server->publish(msg);

        return GST_FLOW_OK;
      }

      void publish( const audio_common_msgs::AudioData &msg )
      {
        _pub.publish(msg);
      }

      static gboolean onMessage (GstBus *bus, GstMessage *message, gpointer userData)
      {
        RosFloatToGst *server = reinterpret_cast<RosFloatToGst*>(userData);
        GError *err;
        gchar *debug;

        gst_message_parse_error(message, &err, &debug);
        ROS_ERROR_STREAM("gstreamer: " << err->message);
        g_error_free(err);
        g_free(debug);
        g_main_loop_quit(server->_loop);
        server->exitOnMainThread(1);
        return FALSE;
      }

			static void cb_need_data (GstElement *appsrc,
					guint       unused_size,
					gpointer    user_data)
			{
				ROS_DEBUG_STREAM("need-data signal emitted! Pausing the pipeline");
				RosFloatToGst *client = reinterpret_cast<RosFloatToGst*>(user_data);
				gst_element_set_state(GST_ELEMENT(client->_pipeline), GST_STATE_PAUSED);
				client->_paused = true;
			}

    private:
      ros::NodeHandle _nh;
      ros::Publisher _pub;
      ros::Subscriber _sub;

      boost::thread _gst_thread;

      GstElement* _pipeline;
      GstElement* _source;
      GstElement* _audioconvert;
      GstElement* _audioresample;
      // GstElement* _filter;
      GstElement* _encode;
      GstElement* _sink;

      GstBus *_bus;
      int _bitrate, _channels, _depth, _sample_rate;
      GMainLoop *_loop;
      std::string _format;

			bool _paused;
  };
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "audio_capture");
  gst_init(&argc, &argv);

  audio_transport::RosFloatToGst server;
  ros::spin();
  // server.spin();
}
