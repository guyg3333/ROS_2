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

#include "subscriber_rec.h"
#include "gStreamLib.h"

using std::placeholders::_1;

RecSub::RecSub() : Node("record_sub")
{
  /* Set QoS */
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  /* Subsribe to video topic */
  this->subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>(
      "video", 10, std::bind(&RecSub::topic_callback, this, _1));
  curFrameNumber = 0;

}

RecSub::~RecSub()
{

}


void RecSub::initFrame()
{
  gStreamLib *p_gst = (gStreamLib *)p_gstClass;

  /* set memory for Frame  */
  curFrame = (uint8_t *)malloc(p_gst->calcFrameSize() * sizeof(uint8_t));

  /* set memory for Frame  */
  ptr_startEnData = (uint8_t *)malloc(p_gst->getEncFrameSize() * sizeof(uint8_t));


  // /*init File*/
  // outputFile = new std::ofstream("10bit_carmela.bin", std::ios::out | std::ios::binary | std::ios::app);
  // if (!outputFile->is_open())
  // {
  //   std::cout << " Fail to  file" << std::endl;
  // }


}

void RecSub::topic_callback(const tutorial_interfaces::msg::Num &msg)
{
  gStreamLib *p_gst = (gStreamLib *)p_gstClass;

  /* if  we process frame copy data from pkt to frame*/
  if (curFrame)
    memcpy(curFrame + msg.pkt, msg.data.data(), msg.size);

  /* if we get all frame pkt then pass the frame to Pipline */
  if (msg.pkt + msg.size == p_gst->calcFrameSize() || curFrameNumber != msg.frame)
  {
    std::cout << "1" << std::endl;
    /* decode frame to 10bit align */
    uint32_t *ptr_cutCopy = (uint32_t *)curFrame;

    /*pointer to copy buff*/
    uint8_t *ptr_enData = ptr_startEnData;
    uint64_t *ptr_enData_64 = (uint64_t *)ptr_enData;
    size_t count = 0;

    std::ofstream outputFile("10bit_carmela.bin", std::ios::out | std::ios::binary | std::ios::app);
    if(outputFile.is_open())
    {
      std::cout << "is open" << std::endl;
    }

    while ((uint8_t *)ptr_cutCopy < (((uint8_t *)curFrame) + p_gst->calcFrameSize() - 1))
    {
      *ptr_enData_64 = 0;
      *(ptr_enData_64 + 1) = 0;
      *ptr_enData_64 |= ((uint64_t) * (ptr_cutCopy)) << 32;     // take 30bit from 1
      *ptr_enData_64 |= ((uint64_t) * (ptr_cutCopy + 1)) << 2;  // take 30bit from 2
      *ptr_enData_64 |= ((uint64_t) * (ptr_cutCopy + 2)) >> 28; // take first 4bit from 3

       ptr_enData_64++; // incremat ptr

      *ptr_enData_64 |= ((uint64_t) * (ptr_cutCopy + 2)) << 36; // take 26 bit from 3
      *ptr_enData_64 |= ((uint64_t) * (ptr_cutCopy + 3)) << 8;  // take 30 bit from 4

      ptr_enData = ptr_enData + 15; /* increment by 15 byte */
      ptr_enData_64 = (uint64_t *)ptr_enData;

      ptr_cutCopy = ptr_cutCopy + 4; /*imcrement by 16 byte */
      //printf("%p,%p ", (uint8_t *)ptr_cutCopy, curFrame + p_gst->calcFrameSize() - 1);
      //std::cout << "i:" + std::to_string(count) << std::endl;
      count++;
    }

    /*reset curFrame*/
    curFrameNumber = msg.frame;

    // save to file
    if (outputFile.is_open())
    {
      std::cout << "==========================  save frame =====================" << std::endl;
      outputFile.write(reinterpret_cast<const char *>(ptr_startEnData), p_gst->getEncFrameSize());
      outputFile.close();
    }
    else
    {
      std::cout << "==========================  ERROR frame =====================" << std::endl;
    }
  }
}


int main(int argc, char *argv[])
{
  /* init Subsriber node  */
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RecSub>();

  /* init Subsriber Pipline  */
  gStreamLib *hGst = new gStreamLib();
  node.get()->p_gstClass = hGst;
  node.get()->initFrame();

  /* Start Pipline */
  // boost::thread imagePlot = boost::thread(boost::bind(&gStreamLib::SubPipline, hGst));

  rclcpp::spin(node);
  rclcpp::shutdown();
  // hGst->closePipline();
  // imagePlot.join();
  delete hGst;

  return 0;
}
