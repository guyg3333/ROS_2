#ifndef GSTREAMLIB_H
#define GSTREAMLIB_H

#include <gst/gst.h>
#include <queue>
#include <string.h>

class gStreamLib
{
public:
    /**
     * @brief
     *
     */
    gStreamLib();
    ~gStreamLib();

    /**
     * @brief Set the Pipline object
     *
     * @param camPath
     * @param camWidth
     * @param camHight
     * @param argc
     * @param argv
     * @return int
     */
    int initPipline();
    // int initPipline(std::string camPath, uint16_t camWidth, uint16_t camHight);
    // int startPipline();
    int closePipline();
    int getWidth();
    int getHight();
    void setWidth(int w);
    void setHight(int h);
    size_t calcFrameSize();
    uint64_t calcPaketRate();
    uint8_t* getFrame();
    void clearFrame();
    void pushFrame(uint8_t* frame);
    int appsrcPipline();
    void push_data_wrapper();



    /*Queue API*/
    std::queue<void*> frameQ;
    GMutex mutex;




    enum vidoRes
    {
        res_320_240 = 0,
        res_640_480,
    } typedef videoRes_t;

private:
    GstElement *pipeline;
    GstElement *appsink;
    GstElement *appsrc;
    GstElement *camSrc;

    GMainContext *context;
    GMainLoop *loop;
    videoRes_t vRes;

    int width;
    int hight;
    int pixelSize;


};

#endif