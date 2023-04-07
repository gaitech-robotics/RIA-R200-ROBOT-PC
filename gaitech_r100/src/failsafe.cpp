#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "FAILSAFE");
  ros::NodeHandle FS_NH;
  ros::Publisher FS_PUB;
  geometry_msgs::Twist FS_MSG;
  FS_PUB=FS_NH.advertise<geometry_msgs::Twist>("FAILSAFE",1000);

  ros::Rate loop_rate(30);
  
  while(ros::ok())
   {
	FS_MSG.linear.x = 0;
	FS_MSG.angular.z= 0;
	FS_PUB.publish(FS_MSG);

	ros::spinOnce();
	loop_rate.sleep();
   }

}
