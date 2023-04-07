/** Gaitech R100 ROS Logger Wrapper
 * 	Copyright (c) 2021, Gaitech Robotics
 * 	Created on 05 Jun, 2021
 * 		Author: usama
 */
#ifndef _GAITECH_R100_ROSLOGGER_H_
#define _GAITECH_R100_ROSLOGGER_H_

#include <ros/ros.h>
#include <janusrc/JRCInterface.h>
/**
 * 	Logger Remapper
 */
class JanusROSLogger : public JanusRC::internal::_Logger {
public:
	JanusROSLogger();
	virtual ~JanusROSLogger();
	virtual void log(const JanusRC::LogLevel& lvl, const std::string& module, const std::string& msg);
private:
	JanusROSLogger(const JanusROSLogger& cpy) = delete;
	const JanusROSLogger& operator=(const JanusROSLogger& rhs) = delete;
};
#endif // _GAITECH_R100_ROSLOGGER_H_