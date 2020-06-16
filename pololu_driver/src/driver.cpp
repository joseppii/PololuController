/**
*
*  Software License Agreement (BSD)
*  
*  \file       driver.cpp
*  \author     Joseph Piperakis <i.piperakis@gmail.com>
*  \copyright  Copyright (c) 2020, Joseph Piperakis.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to the author i.piperakis@gmail.com
*
*/
#include "pololu_driver/controller.h"
#include "pololu_driver/channel.h"

#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "phylax_teleop_joy_velocity");
  ros::NodeHandle nh("~");

  std::string port = "/dev/ttyACM0";
  int32_t baud = 115200;
  ros::Rate loop_rate(10);

  pololu::Controller motorController(port.c_str(), baud);

 // pololu::Channel channel(0, "~");
  motorController.addChannel(0, "left_wheel", "~");
  

  while (ros::ok()) {
    ROS_DEBUG("Attempting connection to %s at %i baud.", port.c_str(), baud);
    motorController.connect();
    if (motorController.connected()) {
      ros::AsyncSpinner spinner(1);
      spinner.start();
      while (ros::ok()) {
        motorController.spinOnce();
      }
      spinner.stop();      
    } else {
      ROS_DEBUG("Problem connecting to serial device.");
      ROS_ERROR_STREAM_ONCE("Problem connecting to port " << port << ". Trying again every 1 second.");
      sleep(1);
    }  
    ROS_INFO("It's alive!!!!!!\n");
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}