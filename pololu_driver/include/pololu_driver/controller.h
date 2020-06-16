/**
*
*  Software License Agreement (BSD)
*  
*  \file       controller.h
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
#ifndef POLOLU_CONTROLLER
#define POLOLU_CONTROLLER

#include <memory>
#include "ros/ros.h"


namespace serial {
  class Serial;
}

namespace pololu 
{

class Channel;

class Controller : public std::enable_shared_from_this<Controller>  {

private:
	const char *_port;
	uint32_t _baud;
	bool _connected;
	std::unique_ptr<serial::Serial> _serial;

	std::vector<std::unique_ptr<pololu::Channel> >_channels;
	ros::NodeHandle _nh;
	ros::Publisher _pub_status;

	void read();
	void processStatus(std::string msg);
	void processFeedback(std::string msg);
public:

	Controller(const char *port, int baud);
	~Controller();
	int exitSafeStart();
	int setTargetSpeed(int speed);
	int getTargetSpeed(int16_t * value);
	int getVariable(uint8_t variable_id, uint16_t * value);
	int getErrorStatus(uint16_t * value);

	void addChannel(int index, std::string name, std::string nh) ;
	void connect();
	bool connected() { return _connected; };
    void spinOnce() { read(); }

	std::shared_ptr<Controller> get_shared_this() { return shared_from_this(); }
};

} // namespace pololu



#endif