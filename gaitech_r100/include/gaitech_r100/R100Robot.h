/** Gaitech R100 ROS Node
 * 	Copyright (c) 2021, Gaitech Robotics
 * 	Created on 05 Jun, 2021
 * 		Author: usama
 */
#ifndef _GAITECH_R100_ROS_NODE_H_
#define _GAITECH_R100_ROS_NODE_H_

#include <gaitech_r100/R100RosLogger.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

/**
 * 	Settings
 */
struct R100RosSettings {
	R100RosSettings();
	// Configuration File //
	std::string		config_file;
	std::string		port_name;
	bool			run_on_startup;
	///////////////////////////////////////////// parameters for external controller only
	std::string		port_left;
	std::string		port_right;
	bool			dir_left_ccw;
	bool			dir_right_ccw;
	bool			ext_drv_testrun;
	bool			use_external_controller;
	//////////////////////////////////////////////
	// Topics //
	std::string		topic_cmdvel;
	std::string		topic_odom;
	// Services //
	std::string		service_start;
	std::string		service_reboot;
	std::string		service_emergency_stop;
	std::string		service_zero_odom;
	// Frames //
	std::string 	base_frame;
	std::string		world_frame;
	void	LoadParameters( ros::NodeHandle &handle);
};

/**
 * 	ROS Node Class
 */
class RosR100Robot {
public:
	RosR100Robot();
	virtual ~RosR100Robot();
protected:
	ros::NodeHandle 	n;
	ros::Subscriber		sCmdVel;
	ros::Publisher		pOdom;
	ros::ServiceServer	svStart;
	ros::ServiceServer	svStop;
	ros::ServiceServer	svRestart;
	ros::ServiceServer	svOdomReset;
	// Custom Data Types //

	// Settings //
	R100RosSettings	settings;
private:
	JanusROSLogger		logger;
	JanusRC::RobotR100*	robot;
	bool				respond_to_cmdvel;
	// Utility Functions //
	int _odom_seq_no;
	void				zero_odometry();
	void				compute_and_pubish_odometry(const JanusRC::Velocity2D &vel2d);
	// Service Callbacks //
	bool srv_start(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	bool srv_stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	bool srv_restart(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	bool srv_reset_odom(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	// callbacks
	void	cmdvel_callback(const geometry_msgs::TwistConstPtr &cmd_vel);
	void	robot_callback(const JanusRC::Event& event);
	static void _robot_callback(const JanusRC::Event& event, void* user_data);
};
#endif // _GAITECH_R100_ROS_NODE_H_