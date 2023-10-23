#ifndef PUBLISHER_MEMBER_FUNCTION_H
#define PUBLISHER_MEMBER_FUNCTION_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include "tutorial_interfaces/msg/num.hpp"                                            // CHANGE

#include <boost/thread.hpp>

class MinimalPublisher : public rclcpp::Node
{
    public:
        MinimalPublisher();
        void image_loop_quite();
        bool get_loop_state();
        void run_loop();
        void initCamera();
        void cameraCapture();
        void gCameraCapture();

        void* p_gstClass; /*<! pointer to Gstreamer class*/
    
    private:
        void timer_callback();
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr video_publisher_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

        cv::VideoCapture cap;
        size_t count_;
        bool isOpenCV;
        bool isVideo;
        bool isFrameDone;

        bool isFrame;
        uint64_t cpyIndex;
        uint64_t frmSize; 
        uint64_t frmCun;


};

#endif