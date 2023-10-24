
#include <publisher_member_function.h>
#include "gStreamLib.h"

using namespace std::chrono_literals;
int count = 0;



MinimalPublisher::MinimalPublisher() : Node("minimal_publisher"), count_(0)
{
  rclcpp::QoS qos(rclcpp::KeepLast(0));                    
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT); 
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);      

  /* Set Video Topic  */
  video_publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("video", 10);
  cpyIndex = 0;
  frmCun = 0;
}

void MinimalPublisher::set_wall_timer(int64_t dur)
{
  timer_ = this->create_wall_timer(std::chrono::microseconds(dur/2), std::bind(&MinimalPublisher::timer_callback, this));
}


void MinimalPublisher::timer_callback()
{

  gStreamLib *p_gst = (gStreamLib *)p_gstClass;
  auto message = tutorial_interfaces::msg::Num();
  size_t cpySize = 100;

  /*Get frame from Pipline*/
  uint8_t *currFrame = (uint8_t *)p_gst->getFrame(); 

  if (currFrame != NULL)
  {
    /*check if residue*/
    if ((int)(p_gst->calcFrameSize() - (cpyIndex + 100)) < 0)
    {
      cpySize = p_gst->calcFrameSize() - cpyIndex;
      isFrameDone = true;
    }

    /*Set Pkt number and size*/
    message.pkt = cpyIndex;
    message.size = cpySize;

    /* Copy part of the frame to pkt */
    currFrame = currFrame + cpyIndex; 
    cpyIndex = cpyIndex + cpySize;

    std::array<unsigned char, 100> byte_array;
    std::copy(currFrame, currFrame + cpySize, byte_array.begin());

    message.data = byte_array;

    /*Publish the pkt*/
    video_publisher_->publish(message);

    if (isFrameDone)
    {
      isFrameDone = false;
      cpyIndex = 0;
      p_gst->clearFrame();
      message.frame = frmCun;
      frmCun = frmCun + 1;
      std::cout << "Frame : " + std::to_string(message.frame) + " " << std::endl;
      ;
    }
  }
}


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  /* Create Pub node */
  auto node = std::make_shared<MinimalPublisher>();

  /* Create Pub Pipline */
  gStreamLib *hGst = new gStreamLib();
  node.get()->p_gstClass = hGst;

  /* Set Publish message rate */
  int64_t pktRate = (int64_t)hGst->calcPaketRate();
  std::cout << pktRate << std::endl;
  node.get()->set_wall_timer(pktRate);

  /* Start pub Pipline */
  boost::thread imageCapture = boost::thread(boost::bind(&gStreamLib::PubPipline, hGst));

  rclcpp::spin(node);
  rclcpp::shutdown();

  hGst->closePipline();
  imageCapture.join();
  delete hGst;

  return 0;
}
