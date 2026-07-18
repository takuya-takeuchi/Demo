#include <gst/gst.h>
#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
    // 1. Argument validation
    // Expected arguments: [executable] [file_path] [stream_name]
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <file_path> <stream_name>" << std::endl;
        return -1;
    }

    std::string filePath = argv[1];
    std::string streamName = argv[2];

    // 2. Initialize GStreamer
    gst_init(&argc, &argv);

    // 3. Construct the pipeline string
    std::string pipelineStr = "filesrc location=\"" + filePath +
                              "\" ! "
                              "qtdemux name=demux "
                              "demux.video_0 ! queue ! "
                              "h264parse ! "
                              "openh264dec ! "
                              "videoconvert ! "
                              "video/x-raw,format=I420 ! "
                              "openh264enc "
                              "rate-control=bitrate "
                              "bitrate=2000000 "
                              "gop-size=60 ! "
                              "h264parse config-interval=-1 ! "
                              "video/x-h264,stream-format=avc,alignment=au ! "
                              "identity sync=true ! "
                              "kvssink "
                              "stream-name=\"" +
                              streamName +
                              "\" "
                              "log-config=\"./kvs_log_configuration\" "
                              "framerate=30 "
                              "fragment-duration=2000";

    std::cout << "Launching pipeline: " << pipelineStr << std::endl;

    GError* error = nullptr;
    GstElement* pipeline = gst_parse_launch(pipelineStr.c_str(), &error);

    if (error)
    {
        std::cerr << "Failed to create pipeline: " << error->message << std::endl;
        g_error_free(error);
        return -1;
    }

    // 4. Start playing the pipeline
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    std::cout << "Streaming to KVS stream: " << streamName << "..." << std::endl;

    // 5. Wait for messages on the bus (Errors or End-of-Stream)
    GstBus* bus = gst_element_get_bus(pipeline);
    GstMessage* msg =
        gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE, (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

    // 6. Handle the message
    if (msg != nullptr)
    {
        if (msg->type == GST_MESSAGE_ERROR)
        {
            GError* err;
            gchar* debug_info;
            gst_message_parse_error(msg, &err, &debug_info);

            std::cerr << "Streaming Error: " << err->message << std::endl;
            if (debug_info)
            {
                std::cerr << "Debug info: " << debug_info << std::endl;
                g_free(debug_info);
            }
            g_error_free(err);
        }
        gst_message_unref(msg);
    }

    // 7. Clean up resources
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    std::cout << "Streaming session ended." << std::endl;
    return 0;
}
