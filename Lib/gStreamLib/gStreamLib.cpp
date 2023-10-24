#include <iostream>
#include "gStreamLib.h"

/* a call back function, retreive data from Pipline to user queue - used in Publisher*/
static GstFlowReturn new_sample(GstElement *sink, gpointer *data)
{
    GstSample *sample;
    gStreamLib *p_gst = (gStreamLib *)data;

    /* Retrieve the buffer */
    g_signal_emit_by_name(sink, "pull-sample", &sample);
    if (sample)
    {
        g_mutex_lock(&(p_gst->mutex));
        if (p_gst->frameQ.size() < 4)
        {
            GstBuffer *buffer = gst_sample_get_buffer(sample);
            GstMapInfo map;
            gst_buffer_map(buffer, &map, GST_MAP_READ);

            uint8_t *buff_cpy = (uint8_t *)malloc(map.size * sizeof(uint8_t));
            memcpy(buff_cpy, map.data, map.size);
            p_gst->frameQ.push(buff_cpy);

            gst_buffer_unmap(buffer, &map);
        }
        g_mutex_unlock(&(p_gst->mutex));

        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    return GST_FLOW_ERROR;
}


/* This callback triggers when appsrc has enough data */
static void stop_feed(GstElement *source, void *user_data)
{
    gStreamLib *p_gst = (gStreamLib *)user_data;
    if (p_gst->sourceid != 0)
    {
        g_print("Stop feeding\n");
        g_source_remove(p_gst->sourceid);
        p_gst->sourceid = 0;
    }
}

/*Push data from user to pipline - used in Subsriber */
static void push_data(void *user_data)
{
    gStreamLib *p_gst = (gStreamLib *)user_data;
    GstFlowReturn ret;

    gpointer data = (gpointer)p_gst->getFrame();
    if (data)
    {
        GstBuffer *buffer = gst_buffer_new_wrapped_full(GST_MEMORY_FLAG_READONLY, data, p_gst->calcFrameSize(), 0, p_gst->calcFrameSize(), NULL, NULL);
        gStreamLib *p_gst = (gStreamLib *)user_data;

        g_signal_emit_by_name(p_gst->appsrc, "push-buffer", buffer, &ret);

        if (ret != GST_FLOW_OK)
        {
            g_printerr("Failed to push buffer. Error: %s\n", gst_flow_get_name(ret));
        }

        p_gst->clearFrame();
        gst_buffer_unref(buffer);
    }
}

/* a call back function that triger g_idle worker to push data */
static void push_data_2(GstElement *_appsrc, guint arg0, void *user_data)
{
    gStreamLib *p_gst = (gStreamLib *)user_data;
    p_gst->sourceid = g_idle_add((GSourceFunc)push_data, user_data);
}


void gStreamLib::push_data_wrapper()
{
    gpointer p_data = (gpointer)getFrame();
    if (p_data)
    {
        push_data(this);
    }
}

gStreamLib::gStreamLib()
{
    width = 320;
    hight = 240;
    vRes = res_320_240;

    g_mutex_init(&mutex);
}

int gStreamLib::closePipline()
{
    g_main_loop_quit(loop);
    return 0;
}

gStreamLib::~gStreamLib()
{
    g_print("gStreamLib Distractor\n");
}


size_t gStreamLib::calcFrameSize()
{
    size_t ans = 0;
    switch (vRes)
    {
    case res_320_240:
        ans = 102720;
        break;

    case res_640_480:
        ans = 410880;
        break;
    }

    return ans;
}

uint64_t gStreamLib::calcPaketRate()
{
    uint64_t ans;
    uint64_t FrameSize = calcFrameSize();
    uint64_t numOfpktsInFrame = (FrameSize / 100) + (FrameSize % 100 ? 1 : 0);
    ans = 33333 / numOfpktsInFrame; // in usec;
    return ans;
}

int gStreamLib::PubPipline()
{
    // create a context
    context = g_main_context_new();
    loop = g_main_loop_new(context, FALSE);

    gst_init(NULL, NULL);

    pipeline = gst_parse_launch("v4l2src device=/dev/video0 ! capsfilter name=caps_camera caps=\"video/x-raw, width=640, height=480\" ! tee name=t ! queue ! videoconvert ! video/x-raw, format=GRAY10_LE32 ! videoconvert ! autovideosink t. ! videoconvert !  video/x-raw, format=GRAY10_LE32, ! appsink ", nullptr);
    if (!pipeline)
    {
        std::cerr << "Failed to create the GStreamer pipeline." << std::endl;
        return 1;
    }

    /*get appsink element */
    appsink = gst_bin_get_by_name(GST_BIN(pipeline), "appsink0");
    if (!appsink)
    {
        std::cerr << "Failed to retrieve the appsink element." << std::endl;
        return 1;
    }

     /*get caps element */
    GstElement *camera_caps = gst_bin_get_by_name(GST_BIN(pipeline), "caps_camera");
    if (!camera_caps)
    {
        std::cerr << "Failed to retrieve the capsfilter element." << std::endl;
        return 1;
    }

    /* set Camera resulotion */
    g_object_set(camera_caps, "caps", gst_caps_new_simple("video/x-raw", "width", G_TYPE_INT, width, "height", G_TYPE_INT, hight, "framerate", GST_TYPE_FRACTION, 30, 1, nullptr), nullptr);

    /* setting appsink */
    g_signal_connect(appsink, "new-sample", G_CALLBACK(new_sample), this);
    g_object_set(appsink, "emit-signals", TRUE, NULL);

    /* start the pipline */
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    /* hold on loop */
    g_print("enter loop \n");
    g_main_loop_run(loop);

    /* Clousing the pipe */
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    return 0;
}


int gStreamLib::SubPipline()
{
    context = g_main_context_new();
    loop = g_main_loop_new(context, FALSE);

    g_print("init gst \n");
    gst_init(NULL, NULL);
    g_print("done init gst \n");

    pipeline = gst_parse_launch("appsrc name=mysource ! queue ! video/x-raw, format=GRAY10_LE32 , width=320, height=240, framerate=30/1 ! videoconvert ! autovideosink", nullptr);
    if (!pipeline)
    {
        std::cerr << "Failed to create the GStreamer pipeline." << std::endl;
        return 1;
    }

    appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "mysource");
    // Configure appsrc properties
    g_object_set(G_OBJECT(appsrc), "caps",
                 gst_caps_new_simple("video/x-raw",
                                     "format", G_TYPE_STRING, "GRAY10_LE32",
                                     "width", G_TYPE_INT, width,
                                     "height", G_TYPE_INT, hight,
                                     "framerate", GST_TYPE_FRACTION, 30, 1,
                                     nullptr),
                 nullptr);
    g_object_set(G_OBJECT(appsrc), "format", GST_FORMAT_TIME, nullptr);
    g_object_set(G_OBJECT(appsrc), "is-live", TRUE, nullptr);

    // enable appsrc signals
    g_signal_connect(appsrc, "need-data", G_CALLBACK(push_data_2), this);
    g_signal_connect(appsrc, "enough-data", G_CALLBACK(stop_feed), this);

    g_object_set(appsrc, "emit-signals", TRUE, NULL);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    g_print("enter loop \n");
    g_main_loop_run(loop);

    /*Clousing the pipe*/
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    return 0;
}


void gStreamLib::pushFrame(void *frame)
{
    g_mutex_lock(&mutex);
    if (frameQ.size() < 50 && frame != NULL)
    {
        g_print("push %p\n", frame);
        frameQ.push(frame);
    }
    g_mutex_unlock(&mutex);
}

void *gStreamLib::getFrame()
{
    void *ans = NULL;
    g_mutex_lock(&mutex);
    if (!frameQ.empty())
    {
        ans = (void *)frameQ.front();
    }
    g_mutex_unlock(&mutex);

    return ans;
}

void gStreamLib::clearFrame()
{
    void *p_frame = NULL;
    g_mutex_lock(&mutex);
    if (!frameQ.empty())
    {
        p_frame = (void *)frameQ.front();
        frameQ.pop();
        g_print("free %p\n", p_frame);
        if (p_frame)
        {
            free(p_frame);
        }
    }
    g_mutex_unlock(&mutex);
}
