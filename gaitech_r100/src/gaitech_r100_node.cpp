/** Gaitech R100 ROS Node
 * 	Copyright (c) 2021, Gaitech Robotics
 * 	Created on 05 Jun, 2021
 * 		Author: usama
 */
#include <gaitech_r100/R100Robot.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "gaitech_r100_node");
	RosR100Robot	robot;
	ros::spin();
	return 0;
}