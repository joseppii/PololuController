/**
*
*  Software License Agreement (BSD)
*  
*  \file       channel.cpp
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
#include "pololu_driver/channel.h"
#include "pololu_driver/controller.h"

namespace pololu 
{

Channel::Channel(int channel_id, std::string name, std::string nh,Controller* contr) : _channelId(channel_id), _name(name), _nh(nh),_controller(contr) {
	_feedback_pub = _nh.advertise<std_msgs::String>("feedback", 1);
	_command_sub = _nh.subscribe("/phylax_node/cmd_drive", 1, &Channel::cmdCallback, this);
	
	if (_controller != NULL)
	{
		ROS_INFO("Oh yeah exitted safe start!");
	}
	else
	{

		ROS_INFO("Oopppps cannot exit safe start!");
	}
	
}

Channel::~Channel() {

}

void Channel::cmdCallback(const phylax_msgs::Drive::ConstPtr& msg) {

	if (msg->mode == phylax_msgs::Drive::MODE_PWM) {
		float cmd = msg->drivers[_channelId];
		
		ROS_INFO("Value for motor %d is: %f", _channelId, cmd);
		
		int16_t speed = int16_t(cmd);
		int result = _controller->setTargetSpeed(speed);
	} else if (msg->mode == phylax_msgs::Drive::MODE_NONE) {
		ROS_INFO("Communication with input controller lost. Safety stop invoked on channel %d!", _channelId);
		int result = _controller->setTargetSpeed(0);
	}
}

} //namespace pololu
