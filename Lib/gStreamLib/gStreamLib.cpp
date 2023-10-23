#include <iostream>
#include "gStreamLib.h"

/* The appsink has received a buffer */
static GstFlowReturn new_sample(GstElement *sink, gpointer *data)
{
    GstSample *sample;
    gStreamLib *p_gst = (gStreamLib *)data;

    /* Retrieve the buffer */
    g_signal_emit_by_name(sink, "pull-sample", &sample);
    if (sample)
    {
        /* The only thing we do in this example is print a * to indicate a received buffer */
        g_print("*");
        // g_print("size %ld", map.size);

        g_mutex_lock(&(p_gst->mutex));
        if (p_gst->frameQ.size() < 4)
        {
            GstBuffer *buffer = gst_sample_get_buffer(sample);
            GstMapInfo map;
            gst_buffer_map(buffer, &map, GST_MAP_READ);
            // gst_buffer_ref(buffer)
            uint8_t *buff_cpy = (uint8_t *)malloc(map.size * sizeof(uint8_t));
            memcpy(buff_cpy, map.data, map.size);
            p_gst->frameQ.push(buff_cpy);
            g_print("push");
            gst_buffer_unmap(buffer, &map);
        }
        g_mutex_unlock(&(p_gst->mutex));

        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    return GST_FLOW_ERROR;
}

void gStreamLib::pushFrame(uint8_t *frame)
{
    g_mutex_lock(&mutex);
    if (frameQ.size() < 4)
    {
        frameQ.push(frame);
    }
    g_mutex_unlock(&mutex);
}

uint8_t *gStreamLib::getFrame()
{
    uint8_t *ans = NULL;
    g_mutex_lock(&mutex);
    if (!frameQ.empty())
    {
        ans = (uint8_t *)frameQ.front();
    }
    g_mutex_unlock(&mutex);

    return ans;
}

void gStreamLib::clearFrame()
{
    uint8_t *p_frame = NULL;
    g_mutex_lock(&mutex);
    if (!frameQ.empty())
    {
        p_frame = (uint8_t *)frameQ.front();
        frameQ.pop();
        g_print("call to free %p\n",p_frame);
        if(p_frame)
            free(p_frame);
    }
    g_mutex_unlock(&mutex);
}

static void push_data(GstElement *_appsrc, guint arg0, gpointer data)
{
    GstSample *sample;
    gStreamLib *p_gst = (gStreamLib *)data;
    GstCaps *caps;
    GstFlowReturn ret;

    gpointer p_data = p_gst->getFrame();
    if(p_data)
    {
        g_print("push data");
        GstBuffer *buffer = gst_buffer_new_wrapped_full(GST_MEMORY_FLAG_READONLY,p_data, p_gst->calcFrameSize(),0, p_gst->calcFrameSize(),NULL,NULL);

        // g_object_get(G_OBJECT(_appsrc),"caps",&caps,NULL);
        // gst_buffer_set_caps(buffer, caps);
        //GstFlowReturn ret = gst_app_src_push_buffer(_appsrc, buffer);
        g_signal_emit_by_name (_appsrc, "push-buffer", buffer, &ret);

        if (ret != GST_FLOW_OK)
        {
            g_printerr("Failed to push buffer. Error: %s\n", gst_flow_get_name(ret));
        }

        p_gst->frameQ.pop();
        //p_gst->calcFrameSize();
        gst_buffer_unref(buffer);

    }

}

void gStreamLib::push_data_wrapper()
{
    push_data(this->appsrc,0,this);
}

gStreamLib::gStreamLib()
{
    width = 640;
    hight = 480;
    pixelSize = 2;
    vRes = res_640_480;

    g_mutex_init(&mutex);
}

int gStreamLib::getWidth()
{
    return width;
}

int gStreamLib::getHight()
{
    return hight;
}
void gStreamLib::setWidth(int w)
{
    width = w;
}
void gStreamLib::setHight(int h)
{
    hight = h;
}

size_t gStreamLib::calcFrameSize()
{
    size_t ans = 0;
    switch (vRes)
    {
    case res_320_240:
        ans = 410880;
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

// int gStreamLib::initPipline(std::string camPath, uint16_t camWidth, uint16_t camHight)
int gStreamLib::initPipline()
{

    // create a context
    context = g_main_context_new();
    loop = g_main_loop_new(context, FALSE);

    g_print("init gst \n");
    gst_init(NULL, NULL);
    g_print("done init gst \n");

    pipeline = gst_parse_launch("v4l2src device=/dev/video1 ! video/x-raw, width=640, height=480 ! tee name=t ! queue ! videoconvert ! video/x-raw, format=GRAY10_LE32 ! videoconvert ! autovideosink t. ! videoconvert !  video/x-raw, format=GRAY10_LE32, ! appsink ", nullptr);
    if (!pipeline)
    {
        std::cerr << "Failed to create the GStreamer pipeline." << std::endl;
        return 1;
    }

    /*Set the appsink element */
    camSrc = gst_bin_get_by_name(GST_BIN(pipeline), "v4l2src0");
    if (!camSrc)
    {
        std::cerr << "Failed to retrieve the v4l2src element." << std::endl;
        return 1;
    }

    appsink = gst_bin_get_by_name(GST_BIN(pipeline), "appsink0");
    if (!appsink)
    {
        std::cerr << "Failed to retrieve the appsink element." << std::endl;
        return 1;
    }

    g_signal_connect(appsink, "new-sample", G_CALLBACK(new_sample), this);
    g_object_set(appsink, "emit-signals", TRUE, NULL);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    g_print("enter loop \n");
    g_main_loop_run(loop);

    /*Clousing the pipe*/
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    return 0;
}

int gStreamLib::closePipline()
{
    g_main_loop_quit(loop);
    return 0;
}

gStreamLib::~gStreamLib()
{
    g_print("gStreamLib\n");
}

int gStreamLib::appsrcPipline()
{
    context = g_main_context_new();
    loop = g_main_loop_new(context, FALSE);

    g_print("init gst \n");
    gst_init(NULL, NULL);
    g_print("done init gst \n");

    pipeline = gst_parse_launch("appsrc name=mysource ! queue ! video/x-raw, format=GRAY10_LE32 , width=640, height=480, framerate=30/1 ! videoconvert ! autovideosink", nullptr);
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
                                     "width", G_TYPE_INT, 640,
                                     "height", G_TYPE_INT, 480,
                                     "framerate", GST_TYPE_FRACTION, 30, 1,
                                     nullptr),
                 nullptr);
    g_object_set(G_OBJECT(appsrc), "format", GST_FORMAT_TIME, nullptr);
    g_object_set(G_OBJECT(appsrc), "is-live", TRUE, nullptr);

    // enable appsrc signals
    g_signal_connect(appsrc, "need-data", G_CALLBACK(push_data), this);
    g_object_set(appsrc, "emit-signals", TRUE, NULL);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    g_print("enter loop \n");
    g_main_loop_run(loop);

    /*Clousing the pipe*/
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    return 0;
}
