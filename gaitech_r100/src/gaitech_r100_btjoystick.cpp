/** Gaitech R100 ROS BT Joystick node
 * 	Copyright (c) 2021, Gaitech Robotics
 * 	Created on 05 Jun, 2021
 * 		Author: usama
 */
#include <gaitech_r100/R100BTJoy.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "gaitech_r100_btjoystick_node");
	R100BTJoyNode	joystick;
	ros::spin();
	return 0;
}