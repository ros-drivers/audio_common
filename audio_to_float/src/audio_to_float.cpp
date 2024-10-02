#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

#include "audio_common_msgs/AudioData.h"
#include "sensor_msgs/ChannelFloat32.h"

namespace audio_transport
{
  class RosGstToFloat
  {
    public:
      RosGstToFloat()
      {
        GstPad *audiopad;

        int output_sample_rate;
        ros::param::param<int>("~output_sample_rate", output_sample_rate, 16000);

        _sub = _nh.subscribe("audio", 10, &RosGstToFloat::onAudio, this);
        _pub = _nh.advertise<sensor_msgs::ChannelFloat32>("decoded", 10);

        _loop = g_main_loop_new(NULL, false);

        _pipeline = gst_pipeline_new("app_pipeline");
        _source = gst_element_factory_make("appsrc", "app_source");
        gst_bin_add( GST_BIN(_pipeline), _source);

        g_signal_connect(_source, "need-data", G_CALLBACK(cb_need_data),this);

        // http://stackoverflow.com/questions/35310415/how-to-access-data-from-gmemoryoutputstream
        // https://github.com/jojva/gst-plugins-base/blob/master/tests/examples/app/appsink-src.c
        {
          _decoder = gst_element_factory_make("decodebin", "decoder");
          if (_decoder == NULL)
          {
            ROS_ERROR_STREAM("couldn't create _sink");
            return;
          }
          g_signal_connect(_decoder, "pad-added", G_CALLBACK(cb_newpad),this);
          gst_bin_add( GST_BIN(_pipeline), _decoder);
          gst_element_link(_source, _decoder);

          _audio = gst_bin_new("audiobin");
          if (_audio == NULL)
          {
            ROS_ERROR_STREAM("couldn't create _sink");
            return;
          }
          _convert = gst_element_factory_make("audioconvert", "convert");
          if (_convert == NULL)
          {
            ROS_ERROR_STREAM("couldn't create _sink");
            return;
          }
					_resample = gst_element_factory_make("audioresample", "resample");
          if (_resample == NULL)
          {
            ROS_ERROR_STREAM("couldn't create _sink");
            return;
          }
          // TODO(lucasw) what is this for?
          audiopad = gst_element_get_static_pad(_convert, "sink");
          _sink = gst_element_factory_make("appsink", "sink");
          if (_sink == NULL)
          {
            ROS_ERROR_STREAM("couldn't create _sink");
            return;
          }

					{
						// 'caps' -> capabilities
						GstCaps *caps;
						caps = gst_caps_new_simple("audio/x-raw",
																"format", G_TYPE_STRING, "F32LE",
																"channels", G_TYPE_INT, 1,
																"layout", G_TYPE_STRING, "interleaved",
																"channel-mask", GST_TYPE_BITMASK, 0x0000000000000001,
																// "width - bits per sample
																// depth - bits ACTUALLY USED FOR AUDIO per sample
																// You can have 32-bit samples, but in each
																// 32-bit group only 16 or 24 bits
																// will be used.
																// This is to achieve necessary alignment."
																"width",    G_TYPE_INT, 32,
																"depth",    G_TYPE_INT, 32,
																"endianness",    G_TYPE_INT, G_BYTE_ORDER,  // 1234
																"rate",     G_TYPE_INT, output_sample_rate,
																"signed",   G_TYPE_BOOLEAN, TRUE,
																NULL);
						g_object_set( G_OBJECT(_sink), "caps", caps, NULL);
						gst_caps_unref(caps);
					}

          g_object_set (G_OBJECT (_sink), "emit-signals", TRUE, "sync", FALSE, NULL);
          g_signal_connect (_sink, "new-sample",
              G_CALLBACK (on_new_sample_from_sink), this);

          gst_bin_add_many( GST_BIN(_audio), _resample, _convert, _sink, NULL);
          // gst_bin_add_many( GST_BIN(_audio), _convert, _sink, NULL);
          gst_element_link(_convert, _sink);
          gst_element_add_pad(_audio, gst_ghost_pad_new("sink", audiopad));
          gst_object_unref(audiopad);

          gst_bin_add(GST_BIN(_pipeline), _audio);
        }

        gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);

        _gst_thread = boost::thread( boost::bind(g_main_loop_run, _loop) );

        _paused = false;
      }

    private:

      void onAudio(const audio_common_msgs::AudioDataConstPtr &msg)
      {
        // ROS_DEBUG_STREAM("new audio " << msg->data.size());
        if(_paused)
        {
          gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);
          _paused = false;
        }

        GstBuffer *buffer = gst_buffer_new_and_alloc(msg->data.size());
        gst_buffer_fill(buffer, 0, &msg->data[0], msg->data.size());
        GstFlowReturn ret;

        g_signal_emit_by_name(_source, "push-buffer", buffer, &ret);
      }

		/* called when the appsink notifies us that there is a new buffer ready for
						o/ http://fossies.org/linux/gst-plugins-base/tests/examples/app/appsink-src.c
		 * processing */
			static void  // GstFlowReturn
      on_new_sample_from_sink (GstElement * elt, gpointer data)
			{
        ROS_DEBUG_STREAM("new sample");
        RosGstToFloat *client = reinterpret_cast<RosGstToFloat*>(data);
				GstSample *sample;
				GstBuffer *buffer;
				GstElement *source;

				/* get the sample from appsink */
				sample = gst_app_sink_pull_sample (GST_APP_SINK (elt));
				buffer = gst_sample_get_buffer (sample);

        GstMapInfo map;
        if (gst_buffer_map (buffer, &map, GST_MAP_READ))
        {
          // ROS_INFO_STREAM("map "
          //     << map.size << " "
          //     << map.maxsize << " ");
          // gst_util_dump_mem (map.data, map.size);
          sensor_msgs::ChannelFloat32 msg;
          const size_t sz = sizeof(float);
          msg.values.resize(map.size / sz);
          // TODO(lucasw) copy this more efficiently
          for (size_t i = 0; i < map.size / sz; ++i)
          {
            // TODO(lucasw) can this format be assumed from
            // default conversion settings here?
            #if 0
            const int hi = (map.data[i * 2 + 1] + 128) % 256;
            // TODO(lucasw) not sure about the + 128 here
            const int lo = (map.data[i * 2] + 128) % 256;
            msg.values[i] = (static_cast<float>(hi - 127) * 256.0 + lo) /
                static_cast<float>(1 << 15);
            #endif
            msg.values[i] = *(reinterpret_cast<float*>(&map.data[i * sz]));
          }
          gst_buffer_unmap (buffer, &map);
          client->_pub.publish(msg);
        }
        gst_sample_unref (sample);
			}

      static void cb_newpad (GstElement *decodebin, GstPad *pad,
                             gpointer data)
      {
        RosGstToFloat *client = reinterpret_cast<RosGstToFloat*>(data);

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

     static void cb_need_data (GstElement *appsrc,
                   guint       unused_size,
                   gpointer    user_data)
     {
       ROS_DEBUG("need-data signal emitted! Pausing the pipeline");
       RosGstToFloat *client = reinterpret_cast<RosGstToFloat*>(user_data);
       gst_element_set_state(GST_ELEMENT(client->_pipeline), GST_STATE_PAUSED);
       client->_paused = true;
     }

      ros::NodeHandle _nh;
      ros::Subscriber _sub;
      ros::Publisher _pub;
      boost::thread _gst_thread;

      GstElement *_pipeline, *_source, *_sink, *_decoder, *_convert, *_resample, *_audio;
      GMainLoop *_loop;

      bool _paused;
  };
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "audio_play");
  gst_init(&argc, &argv);

  audio_transport::RosGstToFloat client;

  ros::spin();
}
