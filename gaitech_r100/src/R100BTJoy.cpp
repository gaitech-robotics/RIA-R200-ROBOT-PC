/** Gaitech R100 ROS Node for Bluetooth Joystick
 * 	Copyright (c) 2021, Gaitech Robotics
 * 	Created on 05 Jun, 2021
 * 		Author: usama
 */
#include <gaitech_r100/R100BTJoy.h>
#include <std_srvs/Empty.h>

/**
 * 	ROS BT Joystick Node Class
 */
R100BTJoyNode::R100BTJoyNode() : n("~") {
	started = false;
	selected_level = 0;
	sel_fw = 0.0;
	sel_rot = 0.0;
	_tprev_btn_start = ros::Time::now();
	_tprev_btn_emgstop = ros::Time::now();
	_tprev_btn_restart = ros::Time::now();
	_tprev_btn_1 = ros::Time::now();
	_tprev_btn_2 = ros::Time::now();
	_tprev_btn_3 = ros::Time::now();
	_tprev_btn_4 = ros::Time::now();
	_tprev_btn_velup = ros::Time::now();
	_tprev_btn_veldown = ros::Time::now();
	ROS_INFO("Starting R100 BT Joystick Node, Loading from Parameter Server");
	this->settings.LoadParameters(this->n);
	ROS_INFO("%s", this->settings.button_map.to_string().c_str());
	// Publishers
	this->pCmdVel = this->n.advertise<geometry_msgs::Twist>(this->settings.topic_cmdvel, 2);
	ROS_INFO("Will Publish Command Velocity on %s", this->settings.topic_cmdvel.c_str());
	// Subscribers
	this->sJoy = this->n.subscribe<sensor_msgs::Joy>(this->settings.topic_joystick, 2, &R100BTJoyNode::joystick_callback, this);
	ROS_INFO("Subscribed to %s for bluetooth joystick data", this->settings.topic_joystick.c_str());
	// Start Thread //
	this->_pub_thread = std::thread(R100BTJoyNode::_thcallback, this);
	this->started = this->settings.pub_cmdvel_startup;
}
void R100BTJoyNode::joystick_callback(const sensor_msgs::JoyConstPtr &jdata) {
	this->state_current = GamePadState();
	this->state_current.update_state(jdata, this->settings.button_map);
	uint32_t	pad_press = this->state_current.user_pressed(this->state_prev);
	if (pad_press != GamePadPress::ACTION_NONE) {
		//////// Button Presses ///////////
		if ( (pad_press & this->settings.btn_emgstop) && this->state_current.button_state(this->settings.btn_emgstop) && this->_timed_out(this->_tprev_btn_emgstop) ) {
			this->started = false;
			this->_tprev_btn_emgstop = ros::Time::now();
			ros::ServiceClient client = this->n.serviceClient<std_srvs::Empty>(this->settings.service_stop);
			if ( client.exists() ) {
				std_srvs::Empty	_obj;
				if ( client.call(_obj) ) {
					ROS_DEBUG("Emergency Stopped pressed, %s called", this->settings.service_stop.c_str());
				} else {
					ROS_WARN("Service call %s failed", this->settings.service_stop.c_str());
				}
			} else {
				ROS_WARN("Cannot connect to %s", this->settings.service_stop.c_str());
			}
		} else if ( (pad_press & this->settings.btn_start) && this->state_current.button_state(this->settings.btn_start) && this->_timed_out(this->_tprev_btn_start) ) {
			this->started = true;
			this->_tprev_btn_start = ros::Time::now();
			ros::ServiceClient client = this->n.serviceClient<std_srvs::Empty>(this->settings.service_start);
			if ( client.exists() ) {
				std_srvs::Empty	_obj;
				if ( client.call(_obj) ) {
					ROS_DEBUG("Start pressed, %s called", this->settings.service_start.c_str());
				} else {
					ROS_WARN("Service call %s failed", this->settings.service_start.c_str());
				}
			} else {
				ROS_WARN("Cannot connect to %s", this->settings.service_start.c_str());
			}
		} else if ( (pad_press & this->settings.btn_restart) && this->state_current.button_state(this->settings.btn_restart) && this->_timed_out(this->_tprev_btn_restart) ) {
			this->started = false;
			this->_tprev_btn_restart = ros::Time::now();
			ros::ServiceClient client = this->n.serviceClient<std_srvs::Empty>(this->settings.service_restart);
			if ( client.exists() ) {
				std_srvs::Empty	_obj;
				if ( client.call(_obj) ) {
					ROS_DEBUG("Restart pressed, %s called", this->settings.service_restart.c_str());
				} else {
					ROS_WARN("Service call %s failed", this->settings.service_restart.c_str());
				}
			} else {
				ROS_WARN("Cannot connect to %s", this->settings.service_restart.c_str());
			}
		} else if ( (pad_press & this->settings.btn_1) && this->state_current.button_state(this->settings.btn_1) && this->_timed_out(this->_tprev_btn_1) ) {
			this->_tprev_btn_1 = ros::Time::now();
			ros::ServiceClient client = this->n.serviceClient<std_srvs::Empty>(this->settings.service_action1);
			if ( client.exists() ) {
				std_srvs::Empty	_obj;
				if ( client.call(_obj) ) {
					ROS_DEBUG("Action 1 pressed, %s called", this->settings.service_action1.c_str());
				} else {
					ROS_WARN("Service call %s failed", this->settings.service_action1.c_str());
				}
			} else {
				ROS_WARN("Cannot connect to %s", this->settings.service_action1.c_str());
			}
		} else if ( (pad_press & this->settings.btn_2) && this->state_current.button_state(this->settings.btn_2) && this->_timed_out(this->_tprev_btn_2) ) {
			this->_tprev_btn_2 = ros::Time::now();
			ros::ServiceClient client = this->n.serviceClient<std_srvs::Empty>(this->settings.service_action2);
			if ( client.exists() ) {
				std_srvs::Empty	_obj;
				if ( client.call(_obj) ) {
					ROS_DEBUG("Action 2 pressed, %s called", this->settings.service_action2.c_str());
				} else {
					ROS_WARN("Service call %s failed", this->settings.service_action2.c_str());
				}
			} else {
				ROS_WARN("Cannot connect to %s", this->settings.service_action2.c_str());
			}
		} else if ( (pad_press & this->settings.btn_3) && this->state_current.button_state(this->settings.btn_3) && this->_timed_out(this->_tprev_btn_3) ) {
			this->_tprev_btn_3 = ros::Time::now();
			ros::ServiceClient client = this->n.serviceClient<std_srvs::Empty>(this->settings.service_action3);
			if ( client.exists() ) {
				std_srvs::Empty	_obj;
				if ( client.call(_obj) ) {
					ROS_DEBUG("Action 3 pressed, %s called", this->settings.service_action3.c_str());
				} else {
					ROS_WARN("Service call %s failed", this->settings.service_action3.c_str());
				}
			} else {
				ROS_WARN("Cannot connect to %s", this->settings.service_action3.c_str());
			}
		} else if ( (pad_press & this->settings.btn_4) && this->state_current.button_state(this->settings.btn_4) && this->_timed_out(this->_tprev_btn_4) ) {
			this->_tprev_btn_4 = ros::Time::now();
			ros::ServiceClient client = this->n.serviceClient<std_srvs::Empty>(this->settings.service_action4);
			if ( client.exists() ) {
				std_srvs::Empty	_obj;
				if ( client.call(_obj) ) {
					ROS_DEBUG("Action 4 pressed, %s called", this->settings.service_action4.c_str());
				} else {
					ROS_WARN("Service call %s failed", this->settings.service_action4.c_str());
				}
			} else {
				ROS_WARN("Cannot connect to %s", this->settings.service_action4.c_str());
			}
		} else if ( (pad_press & this->settings.btn_vel_up) && this->state_current.button_state(this->settings.btn_vel_up) && this->_timed_out(this->_tprev_btn_velup) ) {
			this->_tprev_btn_velup = ros::Time::now();
			this->selected_level = (this->selected_level+2 > this->settings.speed_levels)? this->selected_level : this->selected_level + 1;
			ROS_DEBUG("Speed selection changed to level %d", this->selected_level);
		} else if ( (pad_press & this->settings.btn_vel_down) && this->state_current.button_state(this->settings.btn_vel_down) && this->_timed_out(this->_tprev_btn_veldown) ) {
			this->_tprev_btn_veldown = ros::Time::now();
			this->selected_level = (this->selected_level-1 < 0)? 0 : this->selected_level - 1;
			ROS_DEBUG("Speed selection changed to level %d", this->selected_level);
		}
		//////////// Stick Movement /////////////////
		if ( (pad_press & (this->settings.ax_fwd | this->settings.ax_rot)) ) {
			float	stick_fwd = this->state_current.axis_state(this->settings.ax_fwd);
			float	stick_rot = this->state_current.axis_state(this->settings.ax_rot);
			// Calculate command speed //
			float 	step_fwd = float(this->selected_level+1)*((this->settings.fwd_vel_max - this->settings.fwd_vel_min) / float(this->settings.speed_levels));
			float 	step_rot = float(this->selected_level+1)*((this->settings.rot_vel_max - this->settings.rot_vel_min) / float(this->settings.speed_levels));
			float	off_fwd = (stick_fwd > 0.0)? this->settings.fwd_vel_min : -this->settings.fwd_vel_min;
			float	off_rot = (stick_rot > 0.0)? this->settings.rot_vel_min : -this->settings.rot_vel_min;
			this->sel_fw = (this->state_current.axis_state(this->settings.ax_fwd) == 0.0f)? 0.0 : (stick_fwd * step_fwd) + off_fwd;
			this->sel_rot = (this->state_current.axis_state(this->settings.ax_rot) == 0.0f)? 0.0 : (stick_rot * step_rot) + off_rot;
		}
		if ( (pad_press & this->settings.ax_pan) ) {
			// TODO
			ROS_INFO("TODO Pan : %f", this->state_current.axis_state(this->settings.ax_pan) );
		}
		if ( (pad_press & this->settings.ax_tilt) ) {
			// TODO
			ROS_INFO("TODO Tilt : %f", this->state_current.axis_state(this->settings.ax_tilt) );
		}
		if ( (pad_press & this->settings.ax_lvl1) ) {
			// TODO
			ROS_INFO("TODO LVL1 : %f", this->state_current.axis_state(this->settings.ax_lvl1) );
		}
		if ( (pad_press & this->settings.ax_lvl2) ) {
			// TODO
			ROS_INFO("TODO LVL2 : %f", this->state_current.axis_state(this->settings.ax_lvl2) );
		}
	}
	this->state_prev = this->state_current;
}
void	R100BTJoyNode::publish_cmdvel(const float &fwd_vel_ms, const float &rot_radps) {
	geometry_msgs::Twist	pub_vel;
	pub_vel.linear.x = fwd_vel_ms;	pub_vel.linear.y = 0.0; pub_vel.linear.z = 0.0;
	pub_vel.angular.z = rot_radps;	pub_vel.angular.x =0.0; pub_vel.angular.y = 0.0;
	this->pCmdVel.publish(pub_vel);	// Publish Command Velocity
}
R100BTJoyNode::~R100BTJoyNode() {
	// Close Up Stuff //
	ROS_INFO("Exiting ROS BT Joystick Node");
}
void	R100BTJoyNode::_th_callback() {
	ros::Rate rate(this->settings.pub_freq);
	while( ros::ok() ) {
		if ( this->started ) {
			this->publish_cmdvel(this->sel_fw, this->sel_rot);
		}
		rate.sleep();
	}
	ROS_INFO("bt_joystick publishing cmd_vel thread exiting");
}
void	R100BTJoyNode::_thcallback(const void* _data) {
	R100BTJoyNode* thisptr = (R100BTJoyNode*)_data;
	if (thisptr != nullptr) thisptr->_th_callback();
}

bool	R100BTJoyNode::_timed_out(const ros::Time&old) const {
	ros::Duration	diff = ros::Time::now() - old;
	if ( diff.toSec() > this->settings.btn_wait ) return true;
	return false;
}
/**
 * 	Settings for Bluetooth Joystick
 */
R100BTJoyRosSettings::R100BTJoyRosSettings() : button_map() {
	// TODO
	this->topic_cmdvel = "cmd_vel";
	this->topic_joystick = "joy";
	// Services //
	this->service_start = "action_start";
	this->service_stop = "action_stop";
	this->service_restart = "action_restart";
	this->service_action1 = "action1";
	this->service_action2 = "action2";
	this->service_action3 = "action3";
	this->service_action4 = "action4";
	// Dummy Default Caps //
	this->pub_cmdvel_startup = false;
	this->pub_freq = 10.0;
	this->fwd_vel_max = 1.0;
	this->fwd_vel_min = 0.2;
	this->rot_vel_max = 0.8;
	this->rot_vel_min = 0.0;
	this->btn_wait = 0.5;
	this->speed_levels = 5;
	// Default Button - Action Mapping //
	this->btn_start = GamePadPress::BTN_Start;
	this->btn_emgstop = GamePadPress::BTN_Back;
	this->btn_restart = GamePadPress::BTN_Logo;
	this->btn_1 = GamePadPress::BTN_A;
	this->btn_2 = GamePadPress::BTN_B;
	this->btn_3 = GamePadPress::BTN_X;
	this->btn_4 = GamePadPress::BTN_Y;
	this->btn_vel_up = GamePadPress::BTN_RB;
	this->btn_vel_down = GamePadPress::BTN_LB;
	this->ax_fwd = GamePadPress::AXS_RV;
	this->ax_rot = GamePadPress::AXS_LH;
	this->ax_pan = GamePadPress::AXS_DH;
	this->ax_tilt = GamePadPress::AXS_DV;
	this->ax_lvl1 = GamePadPress::AXS_LT;
	this->ax_lvl2 = GamePadPress::AXS_RT;
}
void	R100BTJoyRosSettings::LoadParameters( ros::NodeHandle &handle) {
	// Remapping //
	this->topic_cmdvel = ros::names::remap(this->topic_cmdvel);
	this->topic_joystick = ros::names::remap(this->topic_joystick);
	this->service_start = ros::names::remap(this->service_start);
	this->service_stop = ros::names::remap(this->service_stop);
	this->service_restart = ros::names::remap(this->service_restart);
	this->service_action1 = ros::names::remap(this->service_action1);
	this->service_action2 = ros::names::remap(this->service_action2);
	this->service_action3 = ros::names::remap(this->service_action3);
	this->service_action4 = ros::names::remap(this->service_action4);
	// Settings //
	handle.param<bool>("pub_cmdvel_onstartup", this->pub_cmdvel_startup, this->pub_cmdvel_startup);
	handle.param<float>("pub_freq", this->pub_freq, this->pub_freq);
	handle.param<float>("fwd_vel_max", this->fwd_vel_max, this->fwd_vel_max);
	handle.param<float>("fwd_vel_min", this->fwd_vel_min, this->fwd_vel_min);
	handle.param<float>("rot_vel_max", this->rot_vel_max, this->rot_vel_max);
	handle.param<float>("rot_vel_min", this->rot_vel_min, this->rot_vel_min);
	handle.param<float>("button_repeat_wait", this->btn_wait, this->btn_wait);
	handle.param<int>("speed_levels", this->speed_levels, this->speed_levels);
	if (this->speed_levels < 1) {
		this->speed_levels = 1;
		ROS_WARN("Speed Levels set to less than 1 , overriding to 1");
	}
	if (this->btn_wait < 0.0 ) {
		this->btn_wait = 0.0;
		ROS_WARN("Button Repeat Wait set to less than 0, overriding to 0.0");
	}
	if (this->fwd_vel_max < 0.0 ) {
		this->fwd_vel_max = -this->fwd_vel_max;
		ROS_WARN("Maximum Forward Velocity negative, correcting it to %f", this->fwd_vel_max);
	}
	if (this->fwd_vel_min < 0.0) {
		this->fwd_vel_min = -this->fwd_vel_min;
		ROS_WARN("Minimum Forward Velocity negative, correcting it to %f", this->fwd_vel_min);
	}
	if (this->fwd_vel_min > this->fwd_vel_max) {
		this->fwd_vel_min = this->fwd_vel_max;
		ROS_WARN("Minimum Forward Velocity greater than Maximum Forward Velocity, overriding to %f", this->fwd_vel_min);
	}
	if (this->rot_vel_max < 0.0 ) {
		this->rot_vel_max = -this->rot_vel_max;
		ROS_WARN("Maximum Rotational Velocity negative, correcting it to %f", this->rot_vel_max);
	}
	if (this->rot_vel_min < 0.0) {
		this->rot_vel_min = -this->rot_vel_min;
		ROS_WARN("Minimum Rotational Velocity negative, correcting it to %f", this->rot_vel_min);
	}
	if (this->rot_vel_min > this->rot_vel_max) {
		this->rot_vel_min = this->rot_vel_max;
		ROS_WARN("Minimum Rotational Velocity greater than Maximum Rotational Velocity, overriding to %f", this->rot_vel_min);
	}
	// Gamepad Mapping //
	this->button_map.load_configuration_from_parameter_server(handle);
	// Button - Action - Mapping //
	std::string val;
	handle.param<std::string>("Action_StartRobot", val, "None");	this->btn_start = R100BTJoyRosSettings::PressFromStr(val);
	handle.param<std::string>("Action_EmergencyStop", val, "None");	this->btn_emgstop = R100BTJoyRosSettings::PressFromStr(val);
	handle.param<std::string>("Action_Restart", val, "None");		this->btn_restart = R100BTJoyRosSettings::PressFromStr(val);
	handle.param<std::string>("Action_SpeedLvlInc", val, "None");	this->btn_vel_up = R100BTJoyRosSettings::PressFromStr(val);
	handle.param<std::string>("Action_SpeedLvlDec", val, "None");	this->btn_vel_down = R100BTJoyRosSettings::PressFromStr(val);
	handle.param<std::string>("Action_1", val, "None");				this->btn_1 = R100BTJoyRosSettings::PressFromStr(val);
	handle.param<std::string>("Action_2", val, "None");				this->btn_2 = R100BTJoyRosSettings::PressFromStr(val);
	handle.param<std::string>("Action_3", val, "None");				this->btn_3 = R100BTJoyRosSettings::PressFromStr(val);
	handle.param<std::string>("Action_4", val, "None");				this->btn_4 = R100BTJoyRosSettings::PressFromStr(val);
	handle.param<std::string>("Action_Forward", val, "None");		this->ax_fwd = R100BTJoyRosSettings::PressFromStr(val);
	handle.param<std::string>("Action_Rotate", val, "None");		this->ax_rot = R100BTJoyRosSettings::PressFromStr(val);
	handle.param<std::string>("Action_Pan", val, "None");			this->ax_pan = R100BTJoyRosSettings::PressFromStr(val);
	handle.param<std::string>("Action_Tilt", val, "None");			this->ax_tilt = R100BTJoyRosSettings::PressFromStr(val);
	handle.param<std::string>("Action_Slider1", val, "None");		this->ax_lvl1 = R100BTJoyRosSettings::PressFromStr(val);
	handle.param<std::string>("Action_Slider2", val, "None");		this->ax_lvl2 = R100BTJoyRosSettings::PressFromStr(val);
}
GamePadPress	R100BTJoyRosSettings::PressFromStr(const std::string& btn_val) {
	if ( btn_val == "Button_Start") return GamePadPress::BTN_Start;
	if ( btn_val == "Button_Back") return GamePadPress::BTN_Back;
	if ( btn_val == "Button_Logo") return GamePadPress::BTN_Logo;
	if ( btn_val == "Button_RB") return GamePadPress::BTN_RB;
	if ( btn_val == "Button_LB") return GamePadPress::BTN_LB;
	if ( btn_val == "Button_A") return GamePadPress::BTN_A;
	if ( btn_val == "Button_B") return GamePadPress::BTN_B;
	if ( btn_val == "Button_X") return GamePadPress::BTN_X;
	if ( btn_val == "Button_Y") return GamePadPress::BTN_Y;
	if ( btn_val == "Button_LA") return GamePadPress::BTN_LA;
	if ( btn_val == "Button_RA") return GamePadPress::BTN_RA;
	if ( btn_val == "Axis_LAnalog_Horizontal") return GamePadPress::AXS_LH;
	if ( btn_val == "Axis_LAnalog_Vertical") return GamePadPress::AXS_LV;
	if ( btn_val == "Axis_RAnalog_Horizontal") return GamePadPress::AXS_RH;
	if ( btn_val == "Axis_RAnalog_Vertical") return GamePadPress::AXS_RV;
	if ( btn_val == "Axis_DPad_Horizontal") return GamePadPress::AXS_DH;
	if ( btn_val == "Axis_DPad_Vertical") return GamePadPress::AXS_DV;
	if ( btn_val == "Axis_LT") return GamePadPress::AXS_LT;
	if ( btn_val == "Axis_RT") return GamePadPress::AXS_RT;
	return GamePadPress::ACTION_NONE;
}
void GamePadConfiguration::load_configuration_from_parameter_server(ros::NodeHandle &handle) {
	// Load Button Configurations //
	handle.param<int>("Button_A", this->btn_A, this->btn_A);
	handle.param<int>("Button_B", this->btn_B, this->btn_B);
	handle.param<int>("Button_X", this->btn_X, this->btn_X);
	handle.param<int>("Button_Y", this->btn_Y, this->btn_Y);
	handle.param<int>("Button_LB", this->btn_LB, this->btn_LB);
	handle.param<int>("Button_RB", this->btn_RB, this->btn_RB);
	handle.param<int>("Button_Back", this->btn_Back, this->btn_Back);
	handle.param<int>("Button_Start", this->btn_Start, this->btn_Start);
	handle.param<int>("Button_Logo", this->btn_Logo, this->btn_Logo);
	handle.param<int>("Button_LAnalog", this->btn_LAnalog, this->btn_LAnalog);
	handle.param<int>("Button_RAnalog", this->btn_RAnalog, this->btn_RAnalog);
	handle.param<int>("Axis_LAnalog_Horizontal", this->axis_LHorz, this->axis_LHorz);
	handle.param<int>("Axis_LAnalog_Vertical", this->axis_LVert, this->axis_LVert);
	handle.param<int>("Axis_RAnalog_Horizontal", this->axis_RHorz, this->axis_RHorz);
	handle.param<int>("Axis_RAnalog_Vertical", this->axis_RVert, this->axis_RVert);
	handle.param<int>("Axis_DPad_Horizontal", this->axis_DHorz, this->axis_DHorz);
	handle.param<int>("Axis_DPad_Vertical", this->axis_DVert, this->axis_DVert);
	handle.param<int>("Axis_LT", this->axis_LT, this->axis_LT);
	handle.param<int>("Axis_RT", this->axis_RT, this->axis_RT);
}
std::string GamePadConfiguration::to_string() const {
	std::string res("Button Configurations\n");
	res += "Buttons A, B, X, Y: " + std::to_string(this->btn_A) + " ," + std::to_string(this->btn_B) + " ,";
	res += std::to_string(this->btn_X) + " ," + std::to_string(this->btn_Y) + "\n";
	res += "LB, RB, LA, RA    : " + std::to_string(this->btn_LB) + " ," + std::to_string(this->btn_RB) + " ,";
	res += std::to_string(this->btn_LAnalog) + " ," + std::to_string(this->btn_RAnalog) + "\n";
	res += "Back, Start, Logo : " + std::to_string(this->btn_Back) + " ," + std::to_string(this->btn_Start);
	res += " ," + std::to_string(this->btn_Logo) + "\n";
	res += "Axis Left Stick | Right Stick : " + std::to_string(this->axis_LHorz) + " ,";
	res += std::to_string(this->axis_LVert) + " | " + std::to_string(this->axis_RHorz) + " , ";
	res += std::to_string(this->axis_RVert) + "\n";
	res += "Axis DPAD | LThumb | RThumb   : " + std::to_string(this->axis_DHorz) + " ," + std::to_string(this->axis_DVert);
	res += " | " + std::to_string(this->axis_LT) + " | " + std::to_string(this->axis_RT);
	return res;
}
GamePadConfiguration::GamePadConfiguration() {
	// Default Button Configuration //
	this->btn_A = 0;
	this->btn_B = 1;
	this->btn_X = 2;
	this->btn_Y = 3;
	this->btn_LB = 4;
	this->btn_RB = 5;
	this->btn_Back = 6;
	this->btn_Start = 7;
	this->btn_Logo = 8;
	this->btn_LAnalog = 9;
	this->btn_RAnalog = 10;
	this->axis_LHorz = 0;
	this->axis_LVert = 1;
	this->axis_RHorz = 3;
	this->axis_RVert = 4;
	this->axis_DHorz = 6;
	this->axis_DVert = 7;
	this->axis_LT = 2;
	this->axis_RT = 5;
}
/**
 * 	Game Pad State
 */
void GamePadState::update_state(const sensor_msgs::JoyConstPtr &data, const GamePadConfiguration& config) {
	// Update GamePad State //
	if (config.btn_A >= 0 && config.btn_A < data->buttons.size())	this->btn_A = (data->buttons[config.btn_A] == 0)? false : true;
	if (config.btn_B >= 0 && config.btn_B < data->buttons.size())	this->btn_B = (data->buttons[config.btn_B] == 0)? false : true;
	if (config.btn_X >= 0 && config.btn_X < data->buttons.size())	this->btn_X = (data->buttons[config.btn_X] == 0)? false : true;
	if (config.btn_Y >= 0 && config.btn_Y < data->buttons.size())	this->btn_Y = (data->buttons[config.btn_Y] == 0)? false : true;
	if (config.btn_LB >= 0 && config.btn_LB < data->buttons.size())	this->btn_LB = (data->buttons[config.btn_LB] == 0)? false : true;
	if (config.btn_RB >= 0 && config.btn_RB < data->buttons.size())	this->btn_RB = (data->buttons[config.btn_RB] == 0)? false : true;
	if (config.btn_LAnalog >= 0 && config.btn_LAnalog < data->buttons.size())	this->btn_LAnalog = (data->buttons[config.btn_LAnalog] == 0)? false : true;
	if (config.btn_RAnalog >= 0 && config.btn_RAnalog < data->buttons.size())	this->btn_RAnalog = (data->buttons[config.btn_RAnalog] == 0)? false : true;
	if (config.btn_Back >= 0 && config.btn_Back < data->buttons.size())	this->btn_Back = (data->buttons[config.btn_Back] == 0)? false : true;
	if (config.btn_Start >= 0 && config.btn_Start < data->buttons.size())	this->btn_Start = (data->buttons[config.btn_Start] == 0)? false : true;
	if (config.btn_Logo >= 0 && config.btn_Logo < data->buttons.size())	this->btn_Logo = (data->buttons[config.btn_Logo] == 0)? false : true;
	if (config.axis_LHorz >= 0 && config.axis_LHorz < data->axes.size()) this->axis_LHorz = data->axes[config.axis_LHorz];
	if (config.axis_RHorz >= 0 && config.axis_RHorz < data->axes.size()) this->axis_RHorz = data->axes[config.axis_RHorz];
	if (config.axis_LVert >= 0 && config.axis_LVert < data->axes.size()) this->axis_LVert = data->axes[config.axis_LVert];
	if (config.axis_RVert >= 0 && config.axis_RVert < data->axes.size()) this->axis_RVert = data->axes[config.axis_RVert];
	if (config.axis_LT >= 0 && config.axis_LT < data->axes.size()) this->axis_LT = data->axes[config.axis_LT];
	if (config.axis_RT >= 0 && config.axis_RT < data->axes.size()) this->axis_RT = data->axes[config.axis_RT];
	if (config.axis_DHorz >= 0 && config.axis_DHorz < data->axes.size()) this->axis_DHorz = data->axes[config.axis_DHorz];
	if (config.axis_DVert >= 0 && config.axis_DVert < data->axes.size()) this->axis_DVert = data->axes[config.axis_DVert];
}
std::string GamePadState::to_string() const {
	// For Diagnostics //
	std::string res("Buttons : ");
	if ( this->btn_A) res += "A ";
	if ( this->btn_B) res += "B ";
	if ( this->btn_X) res += "X ";
	if ( this->btn_Y) res += "Y ";
	if ( this->btn_LB) res += "LB ";
	if ( this->btn_RB) res += "RB ";
	if ( this->btn_Back) res += "Back ";
	if ( this->btn_Start) res += "Start ";
	if ( this->btn_Logo) res += "Logo ";
	if ( this->btn_LAnalog) res += "LA ";
	if ( this->btn_RAnalog) res += "RA ";
	// Axis Values //
	res += "\n Left Stick ( " + std::to_string(this->axis_LHorz) + ", " + std::to_string(this->axis_LVert) + ")";
	res += " Right Stick ( " + std::to_string(this->axis_RHorz) + ", " + std::to_string(this->axis_RVert) + ")";
	res += "\n DPAD " + std::to_string(this->axis_DHorz) + " , " + std::to_string(this->axis_DVert);
	res += " LT " + std::to_string(this->axis_LT) + " RT " + std::to_string(this->axis_RT);
	return res;
}
GamePadState::GamePadState() {
	this->btn_A = false;
	this->btn_B = false;
	this->btn_X = false;
	this->btn_Y = false;
	this->btn_LB = false;
	this->btn_RB = false;
	this->btn_Back = false;
	this->btn_Start = false;
	this->btn_Logo = false;
	this->btn_LAnalog = false;
	this->btn_RAnalog = false;
	this->axis_LHorz = 0.0;
	this->axis_LVert = 0.0;
	this->axis_RHorz = 0.0;
	this->axis_RVert = 0.0;
	this->axis_DHorz = 0.0;
	this->axis_DVert = 0.0;
	this->axis_LT = 1.0;
	this->axis_RT = 1.0;
}
GamePadState::GamePadState(const GamePadState& cpy) {
	this->btn_A = cpy.btn_A;
	this->btn_B = cpy.btn_B;
	this->btn_X = cpy.btn_X;
	this->btn_Y = cpy.btn_Y;
	this->btn_LB = cpy.btn_LB;
	this->btn_RB = cpy.btn_RB;
	this->btn_Back = cpy.btn_Back;
	this->btn_Start = cpy.btn_Start;
	this->btn_Logo = cpy.btn_Logo;
	this->btn_LAnalog = cpy.btn_LAnalog;
	this->btn_RAnalog = cpy.btn_RAnalog;
	this->axis_LHorz = cpy.axis_LHorz;
	this->axis_LVert = cpy.axis_LVert;
	this->axis_RHorz = cpy.axis_RHorz;
	this->axis_RVert = cpy.axis_RVert;
	this->axis_DHorz = cpy.axis_DHorz;
	this->axis_DVert = cpy.axis_DVert;
	this->axis_LT = cpy.axis_LT;
	this->axis_RT = cpy.axis_RT;
}
GamePadState::~GamePadState() { }
const GamePadState& GamePadState::operator=(const GamePadState& cpy) {
	this->btn_A = cpy.btn_A;
	this->btn_B = cpy.btn_B;
	this->btn_X = cpy.btn_X;
	this->btn_Y = cpy.btn_Y;
	this->btn_LB = cpy.btn_LB;
	this->btn_RB = cpy.btn_RB;
	this->btn_Back = cpy.btn_Back;
	this->btn_Start = cpy.btn_Start;
	this->btn_Logo = cpy.btn_Logo;
	this->btn_LAnalog = cpy.btn_LAnalog;
	this->btn_RAnalog = cpy.btn_RAnalog;
	this->axis_LHorz = cpy.axis_LHorz;
	this->axis_LVert = cpy.axis_LVert;
	this->axis_RHorz = cpy.axis_RHorz;
	this->axis_RVert = cpy.axis_RVert;
	this->axis_DHorz = cpy.axis_DHorz;
	this->axis_DVert = cpy.axis_DVert;
	this->axis_LT = cpy.axis_LT;
	this->axis_RT = cpy.axis_RT;
	return *this;
}
bool	GamePadState::operator!=(const GamePadState& rhs) const {
	if ( this->btn_A != rhs.btn_A ) return true;
	if ( this->btn_B != rhs.btn_B ) return true;
	if ( this->btn_X != rhs.btn_X ) return true;
	if ( this->btn_Y != rhs.btn_Y ) return true;
	if ( this->btn_LB != rhs.btn_LB ) return true;
	if ( this->btn_RB != rhs.btn_RB ) return true;
	if ( this->btn_Back != rhs.btn_Back ) return true;
	if ( this->btn_Start != rhs.btn_Start ) return true;
	if ( this->btn_Logo != rhs.btn_Logo ) return true;
	if ( this->btn_LAnalog != rhs.btn_LAnalog ) return true;
	if ( this->btn_RAnalog != rhs.btn_RAnalog ) return true;
	if ( this->axis_LHorz != rhs.axis_LHorz) return true;
	if ( this->axis_LVert != rhs.axis_LVert) return true;
	if ( this->axis_RHorz != rhs.axis_RHorz) return true;
	if ( this->axis_RVert != rhs.axis_RVert) return true;
	if ( this->axis_DHorz != rhs.axis_DHorz) return true;
	if ( this->axis_DVert != rhs.axis_DVert) return true;
	if ( this->axis_LT != rhs.axis_LT) return true;
	if ( this->axis_RT != rhs.axis_RT) return true;
	return false;
}
uint32_t	GamePadState::user_pressed(const GamePadState &rhs) const {
	uint32_t	sts_change = GamePadPress::ACTION_NONE;
	if ( this->btn_A != rhs.btn_A ) sts_change |= GamePadPress::BTN_A;
	if ( this->btn_B != rhs.btn_B ) sts_change |= GamePadPress::BTN_B;
	if ( this->btn_X != rhs.btn_X ) sts_change |= GamePadPress::BTN_X;
	if ( this->btn_Y != rhs.btn_Y ) sts_change |= GamePadPress::BTN_Y;
	if ( this->btn_LB != rhs.btn_LB ) sts_change |= GamePadPress::BTN_LB;
	if ( this->btn_RB != rhs.btn_RB ) sts_change |= GamePadPress::BTN_RB;
	if ( this->btn_Back != rhs.btn_Back ) sts_change |= GamePadPress::BTN_Back;
	if ( this->btn_Start != rhs.btn_Start ) sts_change |= GamePadPress::BTN_Start;
	if ( this->btn_Logo != rhs.btn_Logo ) sts_change |= GamePadPress::BTN_Logo;
	if ( this->btn_LAnalog != rhs.btn_LAnalog ) sts_change |= GamePadPress::BTN_LA;
	if ( this->btn_RAnalog != rhs.btn_RAnalog ) sts_change |= GamePadPress::BTN_RA;
	if ( this->axis_LHorz != rhs.axis_LHorz) sts_change |= GamePadPress::AXS_LH;
	if ( this->axis_LVert != rhs.axis_LVert) sts_change |= GamePadPress::AXS_LV;
	if ( this->axis_RHorz != rhs.axis_RHorz) sts_change |= GamePadPress::AXS_RH;
	if ( this->axis_RVert != rhs.axis_RVert) sts_change |= GamePadPress::AXS_RV;
	if ( this->axis_DHorz != rhs.axis_DHorz) sts_change |= GamePadPress::AXS_DH;
	if ( this->axis_DVert != rhs.axis_DVert) sts_change |= GamePadPress::AXS_DV;
	if ( this->axis_LT != rhs.axis_LT) sts_change |= GamePadPress::AXS_LT;
	if ( this->axis_RT != rhs.axis_RT) sts_change |= GamePadPress::AXS_RT;
	if ( sts_change != GamePadPress::ACTION_NONE) sts_change &= ~(GamePadPress::ACTION_NONE);
	return sts_change;
}
bool	GamePadState::button_state(const GamePadPress& pval) const {
	switch (pval) {
		case GamePadPress::BTN_A:
			return this->btn_A;
		case GamePadPress::BTN_B:
			return this->btn_B;
		case GamePadPress::BTN_X:
			return this->btn_X;
		case GamePadPress::BTN_Y:
			return this->btn_Y;
		case GamePadPress::BTN_Start:
			return this->btn_Start;
		case GamePadPress::BTN_Back:
			return this->btn_Back;
		case GamePadPress::BTN_Logo:
			return this->btn_Logo;
		case GamePadPress::BTN_LB:
			return this->btn_LB;
		case GamePadPress::BTN_RB:
			return this->btn_RB;
		case GamePadPress::BTN_LA:
			return this->btn_LAnalog;
		case GamePadPress::BTN_RA:
			return this->btn_RAnalog;
		default:
			return false;
	}
}
float	GamePadState::axis_state(const GamePadPress& pval) const {
	switch (pval) {
		case GamePadPress::AXS_LH:
			return this->axis_LHorz;
		case GamePadPress::AXS_RH:
			return this->axis_RHorz;
		case GamePadPress::AXS_LV:
			return this->axis_LVert;
		case GamePadPress::AXS_RV:
			return this->axis_RVert;
		case GamePadPress::AXS_DH:
			return this->axis_DHorz;
		case GamePadPress::AXS_DV:
			return this->axis_DVert;
		case GamePadPress::AXS_LT:
			return this->axis_LT;
		case GamePadPress::AXS_RT:
			return this->axis_RT;
		default:
			return 0.0;
	}
}
