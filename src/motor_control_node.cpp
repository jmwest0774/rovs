#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include <sstream>
#include <iostream>
#include <math.h>
#define DEBUG 0
using namespace std;

ros::Publisher steer_pub;
ros::Publisher speed_pub;

/**
	Convert cmd_vel Twist message into steer angles and wheel speeds

	wheel commands are mapped to Twist messages as follows:

	linear.x = left front       angular.x = right front
	linear.y = left center      angular.y = right center
	linear.z = left rear        angular.z = right rear

 */
float limit(float val, float limit)
{
	if(val > limit) return limit;
	if(val < -limit) return -limit;
	return val;
}
	
void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	

	// locations of the wheels relative to the center
	const float front_x = 0.27;   // distance forward to front wheels
	const float rear_x = 0.26;    // distance back to rear wheels
	const float center_y = 0.14;  // sideways distance to center wheels
	const float front_y = 0.12;   // sideways distance to front wheels
	const float rear_y = 0.12;    // sideways distance to rear wheels
	const float wheel_radius = 0.065; // wheel radius
	const float min_omega = 0.05;
	const float min_turn_radius = 0.5;

	float velocity = msg->linear.x;
	float omega = msg->angular.z;

// hack just to get it to work simply
	float steer_angle = omega;
	float wheel_speed = velocity/wheel_radius;
	// calculate steering angle for each wheel
	geometry_msgs::Twist steer;
	steer.linear.x = omega;
	steer.linear.z = -omega;
	steer.angular.x = omega;
	steer.angular.z = -omega;
	steer_pub.publish(steer);

	geometry_msgs::Twist speed;
	speed.linear.x = wheel_speed;
	speed.linear.y = wheel_speed;
	speed.linear.z = wheel_speed;
	speed.angular.x = wheel_speed;
	speed.angular.y = wheel_speed;
	speed.angular.z = wheel_speed;

	speed_pub.publish(speed);
	return;
// end hack
#if 0
	// first check for impossible conditions
	// if omega is very small it can cause problems so just go straight
	if(fabs(omega) < min_omega)
	{
		// calculate steering angle for each wheel.  all zeros for going forward
		geometry_msgs::Twist steer;
		steer_pub.publish(steer);

		// calculate angular velocity for each wheel
		float vel = velocity/wheel_radius;
		geometry_msgs::Twist speed;
		speed.linear.x = vel;
		speed.linear.y = vel;
		speed.linear.z = vel;
		speed.angular.x = vel;
		speed.angular.y = vel;
		speed.angular.z = vel;

		speed_pub.publish(speed);
		return;
	}
	/** turning radius to center of robot
		take absolute value of velocity since radius does not care 
		if forward or backward.  But use sign of omega to determine if
		radius is to left (positive) or right (negative)
		*/
	float r0 = fabs(velocity) / omega;
	
	// rover has a minimum turn radius so don't go beyond it
	r0 = limit(r0, min_turn_radius);

	// calculate turning radius for each wheel    
	float r_lc = r0 - center_y;     // turning radius to left center wheel
	float r_rc = r0 + center_y;     // turning radius to right center wheel
	float r_lf = sqrt((r0-front_y)*(r0-front_y)+ front_x*front_x); // radius to left front 
	float r_lr = sqrt((r0-rear_y)*(r0-rear_y) + rear_x*rear_x); // radius to left rear
	float r_rf = sqrt((r0+front_y)*(r0+front_y) + front_x*front_x);
	float r_rr = sqrt ((r0+rear_y)*(r0+rear_y) + rear_x*rear_x);

	// calculate steering angle for each wheel
	geometry_msgs::Twist steer;
	steer.linear.x = atan2(front_x , r0-front_y);
	steer.linear.z = atan2(-rear_x , r0-rear_y);
	steer.angular.x = atan2(front_x , r0+front_y);
	steer.angular.z = atan2(-rear_x , r0+rear_y);
	steer_pub.publish(steer);

	/* calculate angular velocity for each wheel.
		for velocity, we need to consider sign of velocity but
		not omega. since r0 and r_lf will have same signs, they will cancel
		and only sign of velocity will matter.
	*/
	geometry_msgs::Twist speed;
	speed.linear.x = velocity * r_lf/r0 / wheel_radius;
	speed.linear.y = velocity * r_lc/r0/ wheel_radius;
	speed.linear.z = velocity * r_lr/r0 / wheel_radius;
	speed.angular.x = velocity * r_rf/r0 / wheel_radius;
	speed.angular.y = velocity * r_rc/r0 / wheel_radius;
	speed.angular.z = velocity * r_rr/r0 / wheel_radius;

	speed_pub.publish(speed);
#endif

#if DEBUG
	cout << "Vel="<<velocity<<" omega=" << omega <<endl;
	cout << "r0=" << r0 << " r_lf=" << r_lf << endl;
	cout << "velocities=" << endl;
	cout << "lf=" << omega_lf << " rf=" << omega_rf << endl;
	cout << "lc=" << omega_lc << " rc=" << omega_rc << endl;
	cout << "lr=" << omega_lr << " rr=" << omega_rr << endl;
	cout << "angles" << endl;
	cout << "lf=" << a_lf << " rf=" << a_rf << endl;
	cout << "lr=" << a_lr << " rr=" << a_rr << endl;
#endif
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_control");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("cmd_vel", 1, cmdCallback);
	speed_pub = n.advertise<geometry_msgs::Twist>("speed", 1);
	steer_pub = n.advertise<geometry_msgs::Twist>("steer", 1);
	ros::spin();

	return 0;
}

