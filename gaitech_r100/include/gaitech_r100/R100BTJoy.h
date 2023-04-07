/** Gaitech R100 ROS Node for Bluetooth Joystick
 * 	Copyright (c) 2021, Gaitech Robotics
 * 	Created on 05 Jun, 2021
 * 		Author: usama
 */
#ifndef _GAITECH_R100BTJOY_ROS_NODE_H_
#define _GAITECH_R100BTJOY_ROS_NODE_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <thread>

/**
 * 	Settings
 */
enum GamePadPress : uint32_t {
	BTN_A 		= 0x00000001,
	BTN_B 		= 0x00000002,
	BTN_X 		= 0x00000004,
	BTN_Y 		= 0x00000008,
	BTN_LB		= 0x00000010,
	BTN_RB		= 0x00000020,
	BTN_Back 	= 0x00000040,
	BTN_Start 	= 0x00000080,
	BTN_Logo 	= 0x00000100,
	BTN_LA 		= 0x00000200,
	BTN_RA 		= 0x00000400,
	AXS_LH 		= 0x00010000,
	AXS_LV		= 0x00020000,
	AXS_RH		= 0x00040000,
	AXS_RV		= 0x00080000,
	AXS_DH 		= 0x00100000,
	AXS_DV		= 0x00200000,
	AXS_LT		= 0x00400000,
	AXS_RT		= 0x00800000,
	BTN_NONE 	= 0x10000000,
	AXS_NONE 	= 0x20000000,
	ACTION_NONE = BTN_NONE | AXS_NONE,
};
struct GamePadConfiguration {
	int				btn_A;
	int				btn_B;
	int				btn_X;
	int				btn_Y;
	int				btn_LB;
	int				btn_RB;
	int				btn_Back;
	int				btn_Start;
	int				btn_Logo;
	int				btn_LAnalog;
	int				btn_RAnalog;
	int				axis_LHorz;
	int				axis_LVert;
	int				axis_RHorz;
	int				axis_RVert;
	int				axis_DHorz;
	int				axis_DVert;
	int				axis_LT;
	int				axis_RT;
	void load_configuration_from_parameter_server(ros::NodeHandle &handle);
	std::string 	to_string() const;
	GamePadConfiguration();
};
struct GamePadState {
	bool			btn_A;
	bool			btn_B;
	bool			btn_X;
	bool			btn_Y;
	bool			btn_LB;
	bool			btn_RB;
	bool			btn_Back;
	bool			btn_Start;
	bool			btn_Logo;
	bool			btn_LAnalog;
	bool			btn_RAnalog;
	float			axis_LHorz;
	float			axis_LVert;
	float			axis_RHorz;
	float			axis_RVert;
	float			axis_DHorz;
	float			axis_DVert;
	float			axis_LT;
	float			axis_RT;
	// Constructors / Destructors Operators //
	GamePadState();
	GamePadState(const GamePadState& cpy);
	~GamePadState();
	const GamePadState& operator=(const GamePadState& rhs);
	std::string to_string() const;
	bool	operator!=(const GamePadState& rhs) const;
	inline bool operator==(const GamePadState& rhs) const {
		return !(*this != rhs);
	}
	// Update State //
	void		update_state(const sensor_msgs::JoyConstPtr &data, const GamePadConfiguration& config);
	uint32_t	user_pressed(const GamePadState &press) const;
	bool		button_state(const GamePadPress& pval) const;
	float		axis_state(const GamePadPress& pval) const;
};

struct R100BTJoyRosSettings {
	R100BTJoyRosSettings();
	void	LoadParameters( ros::NodeHandle &handle);
	// Settings //
	bool					pub_cmdvel_startup;
	float					pub_freq;
	float					fwd_vel_max;
	float					fwd_vel_min;
	float					rot_vel_max;
	float					rot_vel_min;
	float					btn_wait;
	int						speed_levels;
	// Button Mapping for BTJoystick Node //
	GamePadPress			btn_start;
	GamePadPress			btn_emgstop;
	GamePadPress			btn_restart;
	GamePadPress			btn_1;
	GamePadPress			btn_2;
	GamePadPress			btn_3;
	GamePadPress			btn_4;
	GamePadPress			btn_vel_up;
	GamePadPress			btn_vel_down;
	GamePadPress			ax_fwd;
	GamePadPress			ax_rot;
	GamePadPress			ax_pan;
	GamePadPress			ax_tilt;
	GamePadPress			ax_lvl1;
	GamePadPress			ax_lvl2;
	// Button Configurations //
	GamePadConfiguration	button_map;
	// Topics //
	std::string		topic_cmdvel;
	std::string		topic_joystick;
	// Services //
	std::string		service_stop;
	std::string		service_start;
	std::string		service_restart;
	std::string		service_action1;
	std::string		service_action2;
	std::string		service_action3;
	std::string		service_action4;
	// velocity level shift up/ shift down //
	// Try Feedback (not necessarily works with linux , driver issue?)
	static GamePadPress	PressFromStr(const std::string& btn_val);
};

/**
 * 	ROS BT Node Class
 */
class R100BTJoyNode {
// TODO
public:
	R100BTJoyNode();
	virtual ~R100BTJoyNode();
	// TODO
protected:
	ros::NodeHandle			n;
	ros::Publisher			pCmdVel;
	//ros::Publisher			pJoyFBack;
	ros::Subscriber			sJoy;
	// BT Joystick node settings
	R100BTJoyRosSettings	settings;
	// GamePad State //
	GamePadState			state_prev, state_current;
	// Selection State //
	bool					started;
	int						selected_level;
	float					sel_fw, sel_rot;
private:
	// Utility Functions //
	void	joystick_callback(const sensor_msgs::JoyConstPtr &jdata);
	void	publish_cmdvel(const float &fwd_vel_ms, const float &rot_radps);
	void				_th_callback();
	static void			_thcallback(const void* _data);
	std::thread			_pub_thread;
	// Button TimeOuts //
	ros::Time			_tprev_btn_start;
	ros::Time			_tprev_btn_emgstop;
	ros::Time			_tprev_btn_restart;
	ros::Time			_tprev_btn_1;
	ros::Time			_tprev_btn_2;
	ros::Time			_tprev_btn_3;
	ros::Time			_tprev_btn_4;
	ros::Time			_tprev_btn_velup;
	ros::Time			_tprev_btn_veldown;
	bool				_timed_out(const ros::Time&old) const;
};

#endif // _GAITECH_R100BTJOY_ROS_NODE_H_