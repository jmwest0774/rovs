#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#include <sstream>

ros::Publisher cmd_pub;

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	float max_vel = 0.3;	// meter/sec
	float max_turn = 0.9;	// rad/sec
	float vel = msg->axes[1] * max_vel;
	float turn = msg->axes[2] * max_turn;
	
	geometry_msgs::Twist cmd;

	cmd.linear.x = vel;
	cmd.angular.z = turn;
	cmd_pub.publish(cmd);
	
  //ROS_INFO("Forward [%f], Turn [%f]", vel, turn);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_control");
 ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("joy", 1, joyCallback);
	cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::spin();

  return 0;
}
