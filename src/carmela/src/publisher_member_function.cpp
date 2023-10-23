// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <publisher_member_function.h>
#include "gStreamLib.h"

using namespace std::chrono_literals;
int count = 0;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

MinimalPublisher::MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
{

  // Define the deadline in microseconds
  uint32_t deadline_microseconds = 9;

  // Convert the microseconds to nanoseconds


  rclcpp::QoS qos(rclcpp::KeepLast(0));                    // Keep the last 10 messages
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT); // Best-effort reliability
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);      // Volatile durability

  // publisher_ = this->create_publisher<std_msgs::msg::String>("Carmela", 10);
  video_publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("video", 10);

  timer_ = this->create_wall_timer(
      std::chrono::microseconds(8), std::bind(&MinimalPublisher::timer_callback, this));
  isVideo = true;
  // initCamera();
  frmSize = 410880;
  cpyIndex = 0;
  frmCun = 0;
}

void MinimalPublisher::image_loop_quite()
{
  isOpenCV = false;
}

bool MinimalPublisher::get_loop_state()
{
  return isOpenCV;
}

void MinimalPublisher::run_loop()
{
  isOpenCV = true;
}

void MinimalPublisher::initCamera()
{
  if (!cap.open(0))
  {
    std::cout << "Fail to set Camera" << std::endl;
    return;
  }

  cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
}

void MinimalPublisher::cameraCapture()
{
  run_loop();
  while (get_loop_state())
  {
    cv::Mat frame;
    cap >> frame;
    if (frame.empty())
      break;
    cv::imshow("cam2image", frame);
    if (cv::waitKey(10) == 27)
      break; // stop capturing by pressing ESC
    std::cout << "==== capture ======" << std::endl;
  }
  cap.release();
  std::cout << "==== Brake ======" << std::endl;
}

void MinimalPublisher::gCameraCapture()
{
}

void MinimalPublisher::timer_callback()
{
  /*isCurrent Frame Done*/
  /*if no continue paketsize */
  /* else tryTogetFrameFromeQueue */
  /*if there is a frame then set it's as the current frame */
  /*if framesize -index > 100 */
  /*copy 100 byte from index */
  /* copy the rest and set frame done */
  gStreamLib *p_gst = (gStreamLib *)p_gstClass;

  auto message = tutorial_interfaces::msg::Num();
  size_t cpySize = 100;

  uint8_t *currFrame = p_gst->getFrame(); // get frame

  if (currFrame != NULL)
  {
    if ((int)(frmSize - (cpyIndex + 100)) < 0) // check if residu
    {
      cpySize = frmSize - cpyIndex;
      isFrameDone = true;
    }
    message.pkt = cpyIndex;
    message.size = cpySize;
    currFrame = currFrame + cpyIndex; // update the copy location
    cpyIndex = cpyIndex + cpySize;    // update index

    std::array<unsigned char, 100> byte_array;
    std::copy(currFrame, currFrame + cpySize, byte_array.begin());

    message.data = byte_array;
    video_publisher_->publish(message);

    if (isFrameDone)
    {
      isFrameDone = false;
      cpyIndex = 0;
      p_gst->clearFrame();
      message.frame = frmCun;
      frmCun = frmCun + 1;
      std::cout << "pop " + std::to_string(message.frame) + " " << std::endl;
      ;
    }
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>();

  /*Start Tharde for capture video*/
  // boost::thread imageCapture = boost::thread(boost::bind(&MinimalPublisher::cameraCapture,node.get()));
  gStreamLib *hGst = new gStreamLib();
  uint64_t pktRate = hGst->calcPaketRate();
  std::cout << pktRate << std::endl;
  node.get()->p_gstClass = hGst;

  boost::thread imageCapture = boost::thread(boost::bind(&gStreamLib::initPipline, hGst));
  /*Init the camera, get the camera width and hight, and calculate the size of one frame*/
  /*calculate the data rate */
  /* allocate queue */
  /* init the node with the data rate and the queue */
  /* start stream */

  rclcpp::spin(node);
  rclcpp::shutdown();
  // node.get()->image_loop_quite();
  // imageCapture.join();
  hGst->closePipline();
  imageCapture.join();
  delete hGst;

  return 0;
}
