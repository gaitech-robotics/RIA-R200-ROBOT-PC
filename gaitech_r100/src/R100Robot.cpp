/** Gaitech R100 ROS Node
 * 	Copyright (c) 2021, Gaitech Robotics
 * 	Created on 05 Jun, 2021
 * 		Author: usama
 */
#include <gaitech_r100/R100Robot.h>

RosR100Robot::RosR100Robot() : n("~"), logger() , robot(nullptr), respond_to_cmdvel(false), _odom_seq_no(0) {
	ROS_INFO("Starting R100 Robot Node, Loading from Parameter Server");
	this->settings.LoadParameters(this->n);
	// Create Object //
	if (this->settings.use_external_controller ) {
		ROS_INFO("Will Use External Controllers, check your launch files for correct configurations");
		this->robot = new JanusRC::RobotR100(&logger, true);
	} else {
		this->robot = new JanusRC::RobotR100(&logger, false);
	}
	// Services //
	this->svStart = this->n.advertiseService(this->settings.service_start,&RosR100Robot::srv_start, this);
	this->svStop = this->n.advertiseService(this->settings.service_emergency_stop,&RosR100Robot::srv_stop, this);
	this->svRestart = this->n.advertiseService(this->settings.service_reboot,&RosR100Robot::srv_restart, this);
	this->svOdomReset = this->n.advertiseService(this->settings.service_zero_odom,&RosR100Robot::srv_reset_odom, this);
	// Publishers //
	this->pOdom = this->n.advertise<nav_msgs::Odometry>(this->settings.topic_odom, 2);
	ROS_INFO("Will Publish odometry on %s", this->settings.topic_odom.c_str());
	// Subscribers //
	this->sCmdVel = this->n.subscribe<geometry_msgs::Twist>(this->settings.topic_cmdvel, 2, &RosR100Robot::cmdvel_callback, this);
	ROS_INFO("Subscribed to %s for command velocity", this->settings.topic_cmdvel.c_str());
	/////////////// Start Robot ///////////////
	if ( !(this->robot->load_configuration(this->settings.config_file)) ) {
		ROS_ERROR("Configuration File Error, check file %s", this->settings.config_file.c_str());
		ros::shutdown();
	} else {
		this->zero_odometry();
		ROS_INFO("Loaded Configuration %s", this->robot->get_configuration().to_string().c_str());
		ROS_INFO("Minimum forward velocity is %.3f m/s", this->robot->get_configuration().get_minimium_forward_velocity());
		ROS_INFO("Maximum forward velocity is %.3f m/s", this->robot->get_configuration().get_maximum_forward_velocity());
		ROS_INFO("Minimum inplace rotation is %.3f rad/s", this->robot->get_configuration().get_minimum_inplace_omega());
		ROS_INFO("Maximum inplace rotation is %.3f rad/s", this->robot->get_configuration().get_maximum_inplace_omega());
		this->robot->add_handler(JanusRC::EventHandler(RosR100Robot::_robot_callback, this));
		// Start Robot //
		if ( this->settings.run_on_startup) {
			if (this->settings.use_external_controller) {
				ROS_INFO("Starting Robot");
				if (this->robot->start_driving_external(this->settings.port_left, this->settings.port_right,
					 this->settings.dir_left_ccw, this->settings.dir_right_ccw, this->settings.ext_drv_testrun)) {
					ROS_INFO("Robot started");
					this->respond_to_cmdvel = true;
				} else {
					ROS_ERROR("Failed to start robot");
				}
			} else {
				if ( this->robot->start_driving(this->settings.port_name) ) {
					ROS_INFO("Robot started");
					this->respond_to_cmdvel = true;
				} else {
					ROS_ERROR("Failed to start robot");
				}
			}			
		}
	}
}
bool RosR100Robot::srv_start(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
	ROS_INFO("Start Robot initiated from ROS side ...");
	if (this->settings.use_external_controller) {
		if (this->robot->start_driving_external(this->settings.port_left, this->settings.port_right,
				this->settings.dir_left_ccw, this->settings.dir_right_ccw, this->settings.ext_drv_testrun)) {
			ROS_INFO("Robot started");
			this->respond_to_cmdvel = true;
			return true;
		}
	} else {
		if ( this->robot->start_driving(this->settings.port_name) ) {
			ROS_INFO("Robot started");
			this->respond_to_cmdvel = true;
			return true;
		}
	}
	this->respond_to_cmdvel = false;
	ROS_ERROR("Failed to start robot");
	return false;
}
bool RosR100Robot::srv_stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
	ROS_INFO("Emergency Stop initiated from ROS side ...");
	this->robot->emergency_stop();
	this->respond_to_cmdvel = false;
	return true;
}
bool RosR100Robot::srv_restart(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
	if (!(this->settings.use_external_controller) ) {
		this->robot->emergency_stop();
		this->respond_to_cmdvel = false;
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	ROS_INFO("Rebooting Robot");
	this->robot->reboot_robot();
	return true;
}
bool RosR100Robot::srv_reset_odom(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
	this->zero_odometry();
	return true;
}
void	RosR100Robot::cmdvel_callback(const geometry_msgs::TwistConstPtr &cmd_vel) {
	if ( this->respond_to_cmdvel ) {
		this->robot->drive_velocity( JanusRC::Velocity2D(cmd_vel->linear.x, 0.0, cmd_vel->angular.z));
	} else {
		ROS_WARN("Robot not started, ingoring command velocity message");
	}
}
void 	RosR100Robot::robot_callback(const JanusRC::Event& event) {
	if (event.event_type == JanusRC::EventType::ROBOT_MOTOR_CONTROLLER_DATA ) {
		//MotorController::MotorCallbackData data = Event2MotorCallbackData(event);			
		// Nothing to do for now //
	} else if ( event.event_type == JanusRC::EventType::ROBOT_WHEEL_ODOMETRY_DATA ) {
		JanusRC::Velocity2D fb = JanusRC::Event2Velocity2D(event);
		this->compute_and_pubish_odometry(fb);
	}
}
void	RosR100Robot::zero_odometry() {
	ROS_INFO("TODO Reset Odometry not implemented");
}
void 	RosR100Robot::compute_and_pubish_odometry(const JanusRC::Velocity2D &vel2d) {
	// Todo refine odometry calculation and compute position later //
	nav_msgs::Odometry	msg;
	msg.header.seq = ++(this->_odom_seq_no);
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = this->settings.world_frame;
	msg.child_frame_id = this->settings.base_frame;
	// Compute and publish Pose // TODO

	//msg.pose.pose.position.x=12345;

	// Todo correct Twist in correct frame //
	msg.twist.twist.linear.x = vel2d.x;
	msg.twist.twist.linear.y = vel2d.y;
	msg.twist.twist.angular.z = vel2d.omega;
	// Publish Twist Covariance , if needed //

	this->pOdom.publish(msg);
}
void RosR100Robot::_robot_callback(const JanusRC::Event& event, void* user_data) {
	RosR100Robot*	thisptr = (RosR100Robot*)user_data;
	if (thisptr != nullptr) thisptr->robot_callback(event);
}
RosR100Robot::~RosR100Robot() {
	// Close Up Stuff //
	ROS_INFO("Exiting ROS Node");
	delete this->robot;
	this->robot = nullptr;
}
/**
 *  Settings / Parameters
 */
void	R100RosSettings::LoadParameters( ros::NodeHandle &handle) {
	// Load settings from parameter server //
	handle.param<std::string>("robot_device", this->port_name, this->port_name);
	handle.param<std::string>("robot_configuration", this->config_file, this->config_file);
	handle.param<bool>("run_on_startup", this->run_on_startup, this->run_on_startup);
	handle.param<std::string>("baseframe", this->base_frame, this->base_frame);
	handle.param<std::string>("worldframe", this->world_frame, this->world_frame);
	// Parameters for External Controller //
	handle.param<std::string>("robot_device_left", this->port_left, this->port_left);
	handle.param<std::string>("robot_device_right", this->port_right, this->port_right);
	handle.param<bool>("direction_left_ccw", this->dir_left_ccw, this->dir_left_ccw);
	handle.param<bool>("direction_right_ccw", this->dir_right_ccw, this->dir_right_ccw);
	handle.param<bool>("ext_test_run", this->ext_drv_testrun, this->ext_drv_testrun);
	handle.param<bool>("use_external_controller", this->use_external_controller, this->use_external_controller);
	// Remapping //
	this->topic_cmdvel = ros::names::remap(this->topic_cmdvel);
	this->topic_odom = ros::names::remap(this->topic_odom);
	this->service_start = ros::names::remap(this->service_start);
	this->service_reboot = ros::names::remap(this->service_reboot);
	this->service_emergency_stop = ros::names::remap(this->service_emergency_stop);
	this->service_zero_odom = ros::names::remap(this->service_zero_odom);
}
R100RosSettings::R100RosSettings() {
	this->base_frame = "base_link";
	this->world_frame = "/world";

	this->topic_cmdvel = "cmd_vel";
	this->topic_odom = "odom";

	this->service_start = "start";
	this->service_reboot = "reboot";
	this->service_emergency_stop = "stop";
	this->service_zero_odom = "odometry_reset";

	this->config_file = "none";
	this->port_name = "/dev/ttyACM0";
	this->run_on_startup = true;


	this->port_left = "/dev/ttyUSB0";
	this->port_right = "/dev/ttyUSB1";
	this->dir_left_ccw = false;
	this->dir_right_ccw = true;
	this->ext_drv_testrun = false;
	this->use_external_controller = false;
}
