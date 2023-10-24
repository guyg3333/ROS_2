#ifndef SUBSCRIBER_MEMBER_FUNCTION_H
#define SUBSCRIBER_MEMBER_FUNCTION_H

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"
#include <boost/thread.hpp>


class MinimalSubscriber : public rclcpp::Node
{
public:

    /**
     * @brief Construct a new Minimal Subscriber object
     * 
     */
    MinimalSubscriber();

    /**
     * @brief 
     * 
     */
    void initFrame();

    void* p_gstClass;   /*<! Pointer to gStreamLib instance*/
    bool isNewFrame;    /*<! flage for new frame */
    uint8_t* curFrame;
    uint64_t curIndex;
    uint64_t curFrameNumber;



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