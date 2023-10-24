#ifndef GSTREAMLIB_H
#define GSTREAMLIB_H

#include <gst/gst.h>
#include <queue>
#include <string.h>

class gStreamLib
{
public:

    /**
     * @brief Construct a new g Stream Lib object
     * 
     */
    gStreamLib();

    /**
     * @brief Destroy the g Stream Lib object
     * 
     */
    ~gStreamLib();

    /**
     * @brief Create Pipline for Publisher node
     * 
     * @return 0 on successes  
     */
    int PubPipline();

    /**
     * @brief Create Pipline for Subsriber node
     * 
     * @return 0 on successes 
     */
    int SubPipline();

    /**
     * @brief close Pipline
     * 
     * @return int 
     */
    int closePipline();

    /**
     * @brief return the frame size in bytes
     * 
     * @return size_t 
     */
    size_t calcFrameSize();

    /**
     * @brief return the time per packetize frame 
     * 
     * @return uint64_t 
     */
    uint64_t calcPaketRate();

    /*Queue API*/
    void* getFrame();
    void clearFrame();
    void pushFrame(void* frame);
    void push_data_wrapper();
    
    std::queue<void*> frameQ; /*<! queue to hold the frmae */
    GMutex mutex; /*<! mutex to handle the queue */
    
    guint sourceid; /*<! sourceid to handle idle worker*/

    enum vidoRes
    {
        res_320_240 = 0,
        res_640_480,
    } typedef videoRes_t;

    videoRes_t vRes;

    GstElement *pipeline;
    GstElement *appsink;
    GstElement *appsrc;
    GstElement *camSrc;

    GMainContext *context;
    GMainLoop *loop;

private:
    int width; /*<! frame width */
    int hight;  /*<! frame hight */
};

#endif