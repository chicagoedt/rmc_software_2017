#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "roboteq_msgs/Command.h"


roboteq_msgs::Command wheel_velocity;

/** \brief converts a twist to corresponding wheel velocities (left and right) 
 *  \param twist_velocity target velocity given as a twist object.
 *  \return pair of float target velocities (first is left wheel and second is right wheel); a wheels_msg object
 *  takes in a target velocity as a twist, then converts it to velocity for wheels (as a wheels_msg object).
 *  First, the angular z velocity is converted to longitudinal velocity (with opposite sides having opposite values) using the formula:
 *  longitudinal_velocity = rotational_velocity * (track_width ^ 2  + wheel_base ^ 2) / 2;
 *  Then, the wheel_velocity = longitudinal_velocity + linear_x_velocity;
 */
void convertTwistToRoboVelocity(const geometry_msgs::Twist::ConstPtr& twist_velocity)
{
        float longitudinal_velocity;

       // longitudinal_velocity = ((twist_velocity->angular.z) * (TRACK_WIDTH ^ 2 + WHEEL_BASE ^ 2)) / (2 * TRACK_WIDTH);

        wheel_velocity.commanded_velocity = twist_velocity->linear.x;
        //wheel_velocity.right = twist_velocity->linear.x + longitudinal_velocity;

        //wheel_velocity.left = wheel_velocity.right + (2 * longitudinal_velocity);

  //take a look at convertTwistToWheelVelocity in deprecated motor_controller
}

int main(int argc, char** argv)
{  
	ros::init(argc, argv, "Msg_Converter");

	ros::NodeHandle n;

	ros::Publisher wheels_power_pub = n.advertise<roboteq_msgs::Command>("roboteq_driver/cmd", 1);
	ros::Subscriber geometry_msgs_sub = n.subscribe("cmd_vel", 1, convertTwistToRoboVelocity);
	
	
	wheels_power_pub.publish(wheel_velocity);

	ros::spin();
}
