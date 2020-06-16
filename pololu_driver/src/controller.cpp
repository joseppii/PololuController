/**
*
*  Software License Agreement (BSD)
*  
*  \file       controller.cpp
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

#include <string>
#include <iostream>
#include <cstdio>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>

#include "pololu_driver/controller.h"
#include "pololu_driver/channel.h"
#include "serial/serial.h"

using namespace serial;

namespace pololu {


Controller::Controller (const char *port, int baud) : _nh("~"), _connected(false), _port(port), _baud(baud) {}

Controller::~Controller () {}

void Controller::connect() 
{
	int result = 0;
	if (!_serial) 
		_serial = std::make_unique<serial::Serial> (_port, _baud, serial::Timeout::simpleTimeout(500));

	if (_serial->isOpen()) {
		ROS_INFO("Connection with serial on port %s",_port);
		_connected = true;
		result = exitSafeStart();
		return;
	} else {
		_connected = false;
		ROS_INFO("No Connection with serial port %s",_port);
	}

  	ROS_INFO("Motor controller not responding.");	  
}

void Controller::read()
{
	ROS_DEBUG_STREAM_NAMED("serial", "Bytes waiting: " << _serial->available());
}

void Controller::addChannel(int index, std::string name, std::string nh) 
{
	_channels.emplace_back(std::make_unique<pololu::Channel>(index, name, nh, this));
}

int Controller::exitSafeStart()
{
  const uint8_t command = 0x83;

  int result = _serial->write(&command, 1);
  return result;
}
int Controller::setTargetSpeed(int speed)
{
  uint8_t command[3];

  if (speed < 0)
  {
    command[0] = 0x86; // Motor Reverse
    speed = -speed;
  }
  else
  {
    command[0] = 0x85; // Motor Forward
  }
  command[1] = speed & 0x1F;
  command[2] = speed >> 5 & 0x7F;

  return _serial->write(command, sizeof(command));
}

int Controller::getVariable(uint8_t variable_id, uint16_t * value)
{
  uint8_t command[] = { 0xA1, variable_id };
  int result = _serial->write(command, sizeof(command));
  if (result) { return -1; }
  uint8_t response[2];
  ssize_t received = _serial->read(response, sizeof(response));
  if (received < 0) { return -1; }
  if (received != 2)
  {
    fprintf(stderr, "read timeout: expected 2 bytes, got %zu\n", received);
    return -1;
  }
  *value = response[0] + 256 * response[1];
  return 0;
}

int Controller::getTargetSpeed(int16_t * value)
{
  return getVariable(20, (uint16_t *)value);
}

int Controller::getErrorStatus(uint16_t * value)
{
  return getVariable(0, value);
}

} // namespace pololu

