/**
*
*  Software License Agreement (BSD)
*  
*  \file       channel.h
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
#ifndef POLOLU_CHANNEL
#define POLOLU_CHANNEL

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "phylax_msgs/Drive.h"
namespace pololu {

class Controller;

class Channel
{
private:
	ros::NodeHandle _nh;
	int _channelId;
	std::string _name;
	Controller* _controller;
	ros::Subscriber _command_sub;
	ros::Publisher _feedback_pub;

public:
	Channel(int channel_id, std::string name, std::string nh, Controller* contr);
	~Channel();
	void cmdCallback(const phylax_msgs::Drive::ConstPtr& msg);
};



} //namespace pololu
#endif