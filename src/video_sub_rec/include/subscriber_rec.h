#ifndef RECSUB_H
#define RECSUB_H

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"
#include <boost/thread.hpp>

#include <iostream>
#include <bitset>
#include <fstream>


class RecSub : public rclcpp::Node
{
public:

    /**
     * @brief Construct a new Minimal Subscriber object
     * 
     */
    RecSub();

    /**
     * @brief Destroy the Rec Sub:: Rec Sub object
     * 
     */
    ~RecSub();

    /**
     * @brief Construct a new init Frame object
     * 
     */
    void initFrame();

    /**
     * @brief 
     * 
     */
    void clouse_file();



    void* p_gstClass;   /*<! Pointer to gStreamLib instance*/
    bool isNewFrame;    /*<! flage for new frame */
    uint8_t* curFrame;
    uint8_t * ptr_startEnData;

    uint64_t curIndex;
    uint64_t curFrameNumber;

    //std::ofstream* outputFile;



private:

    /**
     * @brief topic_callback
     * 
     * @param msg 
     */
    void topic_callback(const tutorial_interfaces::msg::Num &msg);
    rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_;

};

#endif