#include <atomic>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <gst/app/gstappsink.h>
#include <gst/base/gstbasesink.h>
#include <gst/gst.h>

struct StreamState
{
    std::atomic_bool first_keyframe_seen{ false };
};

static GstPadProbeReturn wait_for_first_keyframe_probe(GstPad* pad, GstPadProbeInfo* info, gpointer user_data)
{
    auto* state = static_cast<StreamState*>(user_data);

    if (state->first_keyframe_seen.load())
    {
        return GST_PAD_PROBE_OK;
    }

    GstBuffer* buffer = gst_pad_probe_info_get_buffer(info);
    if (!buffer)
    {
        return GST_PAD_PROBE_OK;
    }

    const bool is_delta = GST_BUFFER_FLAG_IS_SET(buffer, GST_BUFFER_FLAG_DELTA_UNIT);

    if (is_delta)
    {
        std::cout << "Drop delta frame before first keyframe" << std::endl;
        return GST_PAD_PROBE_DROP;
    }

    std::cout << "First keyframe detected" << std::endl;
    state->first_keyframe_seen.store(true);

    return GST_PAD_PROBE_OK;
}

void save_buffer_to_file(GstBuffer* buffer, const std::string& filename)
{
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ))
        return;

    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        gst_buffer_unmap(buffer, &map);
        return;
    }

    file.write(reinterpret_cast<const char*>(map.data), map.size);
    file.close();
    std::cout << "Saved: " << filename << " (" << map.size << " bytes)" << std::endl;

    gst_buffer_unmap(buffer, &map);
}

static GstFlowReturn on_new_sample(GstAppSink* appsink, gpointer user_data)
{
    std::cout << "on_new_sample" << std::endl;
    GstSample* sample = gst_app_sink_pull_sample(appsink);
    if (!sample)
        return GST_FLOW_ERROR;

    GstBuffer* buffer = gst_sample_get_buffer(sample);

    static int frame_count = 0;
    std::string filename = "output/frame_" + std::to_string(frame_count++) + ".jpg";

    save_buffer_to_file(buffer, filename);

    gst_sample_unref(sample);

    return GST_FLOW_OK;
}

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <rtsp_url>" << std::endl;
        return -1;
    }

    std::string rtsp_url = argv[1];

    // Initialize GStreamer
    gst_init(&argc, &argv);

    std::string pipeline_str = "rtspsrc name=src location=" + rtsp_url +
                               " protocols=tcp latency=500 "
                               "src. ! application/x-rtp,media=video,encoding-name=H264 ! "
                               "rtph264depay ! "
                               "h264parse name=parser config-interval=-1 ! "
                               "avdec_h264 ! "
                               "videoconvert ! "
                               "jpegenc quality=100 ! "
                               "appsink name=mysink emit-signals=true sync=false max-buffers=1 drop=true";
    std::cout << "Pipeline: " << pipeline_str << std::endl;
    std::cout << "Pipeline: " << pipeline_str << std::endl;

    GError* error = nullptr;
    GstElement* pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
    if (error)
    {
        std::cerr << "Pipeline error: " << error->message << std::endl;
        g_error_free(error);
        return -1;
    }

    // Drop frames until the first keyframe is received
    StreamState state;
    GstElement* parser = gst_bin_get_by_name(GST_BIN(pipeline), "parser");
    GstPad* parser_src_pad = gst_element_get_static_pad(parser, "src");
    gst_pad_add_probe(parser_src_pad, GST_PAD_PROBE_TYPE_BUFFER, wait_for_first_keyframe_probe, &state, nullptr);
    gst_object_unref(parser_src_pad);
    gst_object_unref(parser);

    // Get the appsink element from the pipeline
    GstElement* appsink = gst_bin_get_by_name(GST_BIN(pipeline), "mysink");
    if (!appsink)
    {
        std::cerr << "Could not find appsink" << std::endl;
        return -1;
    }

    gst_app_sink_set_emit_signals(GST_APP_SINK(appsink), TRUE);
    gst_base_sink_set_sync(GST_BASE_SINK(appsink), FALSE);

    // Configure callback function
    g_signal_connect(appsink, "new-sample", G_CALLBACK(on_new_sample), nullptr);

    // Start pipeline
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    std::cout << "Streaming started. Press Ctrl+C to stop." << std::endl;

    // Main Loop (Wait while pipeline is running)
    GMainLoop* loop = g_main_loop_new(nullptr, FALSE);
    g_main_loop_run(loop);

    // Cleanup
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(appsink);
    gst_object_unref(pipeline);
    g_main_loop_unref(loop);

    return 0;
}
