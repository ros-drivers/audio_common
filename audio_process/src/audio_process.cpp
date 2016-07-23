#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

#include "audio_common_msgs/AudioData.h"

namespace audio_transport
{
  class RosGstProcess
  {
    public:
      RosGstProcess()
      {
        GstPad *audiopad;

        _sub = _nh.subscribe("audio", 10, &RosGstProcess::onAudio, this);

        _loop = g_main_loop_new(NULL, false);

        _pipeline = gst_pipeline_new("app_pipeline");
        _source = gst_element_factory_make("appsrc", "app_source");
        gst_bin_add( GST_BIN(_pipeline), _source);

        g_signal_connect(_source, "need-data", G_CALLBACK(cb_need_data),this);

        // http://stackoverflow.com/questions/35310415/how-to-access-data-from-gmemoryoutputstream
        // https://github.com/jojva/gst-plugins-base/blob/master/tests/examples/app/appsink-src.c
        {
          _decoder = gst_element_factory_make("decodebin", "decoder");
          g_signal_connect(_decoder, "pad-added", G_CALLBACK(cb_newpad),this);
          gst_bin_add( GST_BIN(_pipeline), _decoder);
          gst_element_link(_source, _decoder);

          _audio = gst_bin_new("audiobin");
          _convert = gst_element_factory_make("audioconvert", "convert");
          audiopad = gst_element_get_static_pad(_convert, "sink");
          _sink = gst_element_factory_make("appsink", "sink");
          g_object_set (G_OBJECT (_sink), "emit-signals", TRUE, "sync", FALSE, NULL);
          g_signal_connect (_sink, "new-sample",
              G_CALLBACK (on_new_sample_from_sink), this);

          gst_bin_add_many( GST_BIN(_audio), _convert, _sink, NULL);
          gst_element_link(_convert, _sink);
          gst_element_add_pad(_audio, gst_ghost_pad_new("sink", audiopad));
          gst_object_unref(audiopad);

          gst_bin_add(GST_BIN(_pipeline), _audio);
        }

        gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);
        //gst_element_set_state(GST_ELEMENT(_playbin), GST_STATE_PLAYING);

        _gst_thread = boost::thread( boost::bind(g_main_loop_run, _loop) );

        _paused = false;
      }

    private:

      void onAudio(const audio_common_msgs::AudioDataConstPtr &msg)
      {
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
			static GstFlowReturn
			on_new_sample_from_sink (GstElement * elt, gpointer data)
			{
        RosGstProcess *client = reinterpret_cast<RosGstProcess*>(data);
				GstSample *sample;
				GstBuffer *app_buffer, *buffer;
				GstElement *source;

				/* get the sample from appsink */
				sample = gst_app_sink_pull_sample (GST_APP_SINK (elt));
				buffer = gst_sample_get_buffer (sample);

				/* make a copy */
				app_buffer = gst_buffer_copy (buffer);
        // TODO(lucasw) do something with the app_buffer- plot it
				/* we don't need the appsink sample anymore */
				gst_sample_unref (sample);

				/* get source and push new buffer */
				source = gst_bin_get_by_name (GST_BIN (client->_sink), "app_source");
				return gst_app_src_push_buffer (GST_APP_SRC (source), app_buffer);
			}

      static void cb_newpad (GstElement *decodebin, GstPad *pad,
                             gpointer data)
      {
        RosGstProcess *client = reinterpret_cast<RosGstProcess*>(data);

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
       ROS_WARN("need-data signal emitted! Pausing the pipeline");
       RosGstProcess *client = reinterpret_cast<RosGstProcess*>(user_data);
       gst_element_set_state(GST_ELEMENT(client->_pipeline), GST_STATE_PAUSED);
       client->_paused = true;
     }

      ros::NodeHandle _nh;
      ros::Subscriber _sub;
      boost::thread _gst_thread;

      GstElement *_pipeline, *_source, *_sink, *_decoder, *_convert, *_audio;
      GstElement *_playbin;
      GMainLoop *_loop;

      bool _paused;
  };
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "audio_play");
  gst_init(&argc, &argv);

  audio_transport::RosGstProcess client;

  ros::spin();
}
