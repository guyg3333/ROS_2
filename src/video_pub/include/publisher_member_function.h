#ifndef PUBLISHER_MEMBER_FUNCTION_H
#define PUBLISHER_MEMBER_FUNCTION_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "tutorial_interfaces/msg/num.hpp"                                          
#include <boost/thread.hpp>

class MinimalPublisher : public rclcpp::Node
{
    public:

        /**
         * @brief Construct a new Minimal Publisher object
         * 
         */
        MinimalPublisher();

        /**
         * @brief Set the wall timer object
         * 
         * @param dur 
         */
        void set_wall_timer(int64_t dur);

        void* p_gstClass; /*<! pointer to Gstreamer class*/
    
    private:
        void timer_callback();
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr video_publisher_;

        size_t count_;
        bool isFrameDone;

        uint64_t cpyIndex;
        uint64_t frmSize; 
        uint64_t frmCun;


};

#endif