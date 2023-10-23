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

#include <functional>
#include <memory>

#include "subscriber_member_function.h"
#include "gStreamLib.h"

using std::placeholders::_1;

MinimalSubscriber::MinimalSubscriber() : Node("minimal_subscriber")
{
  rclcpp::QoS qos(rclcpp::KeepLast(0));                    // Keep the last 10 messages
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT); // Best-effort reliability

  this->subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>(
      "video", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  isNewFrame = true;
  curFrame = (uint8_t *)malloc(410880 * sizeof(uint8_t));
  curFrameNumber = 0;
}

void MinimalSubscriber::topic_callback(const tutorial_interfaces::msg::Num &msg)
{
  gStreamLib *p_gst = (gStreamLib *)p_gstClass;

  // if (msg.pkt + msg.size > p_gst->calcFrameSize())
  // {
  //   std::cout << "Bigger" << std::endl;
  // }
  if (curFrame)
    memcpy(curFrame + msg.pkt, msg.data.data(), msg.size);

  if (msg.pkt + msg.size == p_gst->calcFrameSize())
  {
    std::cout << "Enter to Push frame" << std::endl;
    p_gst->pushFrame(curFrame);
    curFrame = (uint8_t *)malloc(p_gst->calcFrameSize() * sizeof(uint8_t));
    p_gst->push_data_wrapper();
    curFrameNumber = msg.frame;
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>();

  gStreamLib *hGst = new gStreamLib();
  node.get()->p_gstClass = hGst;

  boost::thread imagePlot = boost::thread(boost::bind(&gStreamLib::appsrcPipline, hGst));

  std::cout << "got here" << std::flush;

  rclcpp::spin(node);
  rclcpp::shutdown();
  imagePlot.join();
  delete hGst;

  return 0;
}
