/** Gaitech Logger redirect from janus-rc-cpp
 * 	Copyright (c) 2021, Gaitech Robotics
 * 	Created on 05 Jun, 2021
 * 		Author: usama
 */
#include <gaitech_r100/R100RosLogger.h>

JanusROSLogger::JanusROSLogger() { }
JanusROSLogger::~JanusROSLogger() { }
void JanusROSLogger::log(const JanusRC::LogLevel& lvl, const std::string& module, const std::string& msg) {
	switch ( lvl ) {
	case JanusRC::LogLevel::LOG_LEVEL_TRACE:
	case JanusRC::LogLevel::LOG_LEVEL_DEBUG:
		ROS_DEBUG("[%s] %s", module.c_str(), msg.c_str());
		break;
	case JanusRC::LogLevel::LOG_LEVEL_ERROR:
		ROS_ERROR("[%s] %s", module.c_str(), msg.c_str());
		break;
	case JanusRC::LogLevel::LOG_LEVEL_WARN:
		ROS_WARN("[%s] %s", module.c_str(), msg.c_str());
		break;
	case JanusRC::LogLevel::LOG_LEVEL_INFO:
		ROS_INFO("[%s] %s", module.c_str(), msg.c_str());
		break;
	default:
		break;
	}
}
