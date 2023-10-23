#ifndef SUBSCRIBER_MEMBER_FUNCTION_H
#define SUBSCRIBER_MEMBER_FUNCTION_H

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"
#include <boost/thread.hpp>


class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber();
    void* p_gstClass;
    bool isNewFrame;
    uint8_t* curFrame;
    uint64_t curIndex;
    uint64_t curFrameNumber;



private:
    void topic_callback(const tutorial_interfaces::msg::Num &msg);
    rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_;

};

#endif