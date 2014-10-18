/**
 * @file
 * @author Basheer Subei <basheersubei@gmail.com>
 * @version 1.0
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * http://www.gnu.org/copyleft/gpl.html
 *
 * @todo set the constants using ros parameters in main instead of hard-coding them. 
 *
 * @section DESCRIPTION
 * This is the base controller node for the Scipio robot. It subscribes to cmd_vel (target velocities requested higher up) and publishes (in response) to set_wheels_power using an appropriate value to ramp up to smoothly.
 * It also subscribes to query velocity and position (uses them to calculate appropriate new wheels power to set). 
 *
 * The general flow: main runs and keeps checking if current_velocity and target_velocity are way off. If they are, then call updateWheelsPower (dead-simple P controller) that sends new wheels power message to edcs.
 * Meanwhile, the cmd_vel_callback constantly updates target_velocity and the query_velocity_callback updates the current_velocity.
 *
 * current_velocity is set in query_velocity_callback.
 * target_velocity is set in cmd_vel_callback
 * current_wheels_power is set in updateWheelsPower (after sending a new wheels power message)
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <base_controller/wheels_msg.h> /// query velocity message file
#include <cstring>
#include <string>
#define TRACK_WIDTH 34
#define WHEEL_BASE 21
#define ACCELERATION 1.04f
#define MAX_VELOCITY 2.6f
 
  base_controller::wheels_msg _currentVel;
  base_controller::wheels_msg _targetVel;
  base_controller::wheels_msg current_wheels_power;

  ros::Publisher wheels_power_pub;

/** \brief converts cmd_vel Twist (pair of vectors) to wheels velocity (for each side)
 *  \param twist_velocity target velocity given as a twist object.
 *  \return pair of float target velocities (first is left wheel and second is right wheel)
 *  takes in a target velocity as a twist, then converts it to velocity for wheels (as a wheels_msg object).
 *  First, the angular z velocity is converted to longitudinal velocity (with opposite sides having opposite values) using the formula:
 *  longitudinal_velocity = rotational_velocity * (track_width ^ 2  + wheel_base ^ 2) / 2;
 *  Then, the wheels_velocity = longitudinal_velocity + linear_x_velocity;
 */
base_controller::wheels_msg convert_twist_to_wheels_velocity(geometry_msgs::Twist twist_velocity)
{

  base_controller::wheels_msg wheels_velocity;
  float longitudinal_velocity = twist_velocity.angular.z * (TRACK_WIDTH ^ 2 + WHEEL_BASE ^ 2) / 2;

  //angular z positive is counter-clockwise (turn left). this means right wheel should go forward.
  //right wheel uses positive longitudinal_velocity component
  //left wheel uses negative longitudinal_velocity component

  wheels_velocity.left = -1*longitudinal_velocity + twist_velocity.linear.x;
  wheels_velocity.right = longitudinal_velocity + twist_velocity.linear.x;

  return wheels_velocity;
}

/** \brief uses current velocity to compute appropriate delta power needed to reach target velocity (P controller)
 *  \param current_vel the current velocity of both wheels (as a wheels_msg)
 *  \param target_vel the target velocity we want reach eventually (i.e. that we're ramping up to) 
 *  \return the delta power that will smoothly get us to target velocity (as wheels_msg)
 *  Given the current velocity as a parameter, this method uses a P controller to compute the appropriate delta power to set to in order to smoothly reach a target velocity (2nd parameter).
 *  @todo grab P coefficient from a ROS_parameter instead of hard-coding it.
 *
 *  returns delta power level: (error in velocity/max velocity) >> converted from current velocity to power level percentage
 */
base_controller::wheels_msg calculateDeltaPowerLvl(base_controller::wheels_msg current_vel, base_controller::wheels_msg target_vel)
{
  float p_coefficient = 0.05;
  float left_error = target_vel.left - current_vel.left;
  float right_error = target_vel.right - current_vel.right;

  base_controller::wheels_msg _deltaPowerLevel; 
  _deltaPowerLevel.left = (left_error / MAX_VELOCITY) * p_coefficient;
  _deltaPowerLevel.right = (right_error / MAX_VELOCITY) * p_coefficient;

  return _deltaPowerLevel;
}


/** \brief calculates new power level needed to reach target velocity and sends it to topic
 *  
 *  if current_velocity and target_velocity are not within a threshold, then calculate the delta power level needed to bring us closer to target velocity (simple P controller).
 *  new_wheels_power = current_wheels_power + delta_wheels_power.
 *  @todo use a threshold here (check if currentVel is close to targetVel within a threshold, for example +/- 0.1 m/s) 
 */
  void updateWheelPower()
  {
      ROS_INFO("updating wheel power!");
      base_controller::wheels_msg new_wheels_power;
      base_controller::wheels_msg delta_wheels_power;
      // @todo use a threshold here (check if currentVel is close to targetVel within a threshold, for example +/- 0.1 m/s)
      if(_currentVel.left != _targetVel.left || _currentVel.right != _targetVel.right)
      {
          delta_wheels_power = calculateDeltaPowerLvl(_currentVel, _targetVel); //gets change in wheel power needed to reach target velocity (using a P controller)
      }else
      {
        ROS_INFO("cancelling updateWheelPower");
        return;
      }

      //notice this is POWER levels not velocity
      // new = current + delta

      if ( (current_wheels_power.left + delta_wheels_power.left) >= -1.0f && (current_wheels_power.left + delta_wheels_power.left) <= 1.0f)
      {
        new_wheels_power.left = current_wheels_power.left + delta_wheels_power.left; 
      }
      else
      {
        if (current_wheels_power.left < 0)
        {
          new_wheels_power.left = -1.0f;
        } 
        if (current_wheels_power.left > 0)
        {
          new_wheels_power.left = 1.0f;
        } 
      }
      
      if ( (current_wheels_power.right + delta_wheels_power.right) >= -1.0f && (current_wheels_power.right + delta_wheels_power.right) <= 1.0f)
      {
        new_wheels_power.right = current_wheels_power.right + delta_wheels_power.right;
      }
      else
      {
        if (current_wheels_power.right < 0)
        {
          new_wheels_power.right = -1.0f;
        } 
        if (current_wheels_power.right > 0)
        {
          new_wheels_power.right = 1.0f;
        } 
      }
      
      
      

      wheels_power_pub.publish(new_wheels_power); //send new wheels power level to edcs
      current_wheels_power = new_wheels_power; // set current wheels power to what we just sent to edcs

      ROS_INFO("done updating wheel power!");

  }

/** \brief tests convert_twist_to_wheels_velocity by throwing in a bunch of different twists and printing the returned values
 *
 *
 *  creates fake test_twists and converts them using the convert_twist_to_wheels_velocity then prints results.
 *  first, create a test_twist (and ramp its linear x from -2.6 m/s to 2.6 m/s) while printing results.
 *  then, create a test_twist (and ramp its angular z from -0.1 rad/s to 0.1 rad/s) while printing results.
 *  finally, create a test_twist (and ramp its angular z within ramping the linear x) while printing.
 */
void test_convert_twist_to_wheels_velocity()
{

  geometry_msgs::Twist test_twist;
  base_controller::wheels_msg test_wheels_velocity;
  // test linear x velocity
  for(float i=-2.6; i< 2.6; i += 0.1)
  {
    test_twist.linear.x = i;
    test_wheels_velocity = convert_twist_to_wheels_velocity(test_twist);
    ROS_INFO("test_twist: l[%f 0 0] --> test_wheels_velocity: [%f %f]", i, test_wheels_velocity.left, test_wheels_velocity.right);
  }
  
  // test angular z velocity
  for(float i=-0.1; i< 0.1; i += 0.01)
  {
    test_twist.angular.z = i;
    test_wheels_velocity = convert_twist_to_wheels_velocity(test_twist);
    ROS_INFO("test_twist: a[0 0 %f] --> test_wheels_velocity: [%f %f]", i, test_wheels_velocity.left, test_wheels_velocity.right);
  }

  // test both linear and angular
  for(float i=-2.6; i< 2.6; i += 0.1)
  {
    test_twist.linear.x = i;
    for(float j=-0.1; j<0.1; j+= 0.01){
      test_twist.angular.z = j;

      test_wheels_velocity = convert_twist_to_wheels_velocity(test_twist);
      ROS_INFO("test_twist: l[%f 0 0] a[0 0 %f] --> test_wheels_velocity: [%f %f]", i, j, test_wheels_velocity.left, test_wheels_velocity.right);
    }
  }
  
}

/** \brief receives Twist message from /cmd_vel and converts it to wheel velocity then sets target_velocity to that.
 *  \param vel Twist message (linear and angular target velocity vectors) to move the robot
 *
 *  This method is called whenever a Twist cmd_vel is received (remember, this is a target velocity but as a Twist). It converts this twist to wheel velocity. It sets target_velocity variable to that wheel velocity.
 */
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& vel)
{

  ROS_INFO("I heard: linear [%f, %f, %f]", vel->linear.x, vel->linear.y, vel->linear.z); //print out linear velocity command components

  ROS_INFO("I heard: angular [%f, %f, %f]", vel->angular.x, vel->angular.y, vel->angular.z); //print out angular velocity command components
  
  _targetVel = convert_twist_to_wheels_velocity(*vel); // throw in dereferenced vel (since it's a pointer to a twist)

}

void query_position_callback(base_controller::wheels_msg position_msg)
{

  ROS_INFO("received query_position_msg: [%f %f]", position_msg.left, position_msg.right);

}

/** \brief updates current_velocity when it receives a query_velocity message from edcs
 *  \param velocity_msg message containing current velocity (as a wheels_msg)
 *  called whenever edcs queries velocity and sends us the results. Updates current_velocity variable to whatever message it gets.
 */
void query_velocity_callback(base_controller::wheels_msg velocity_msg)
{

  ROS_INFO("received query_velocity_msg: [%f %f]", velocity_msg.left, velocity_msg.right);

  _currentVel = velocity_msg;

}

/** \brief starts node and topics and services
 *	Starts up the node and creates subscribers (to /cmd_vel and query velocity and position) and publisher (to set_wheels_power).
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller");

  ros::NodeHandle n;
  
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1000, cmd_vel_callback);
  
  ros::Subscriber query_velocity_sub = n.subscribe("query_velocity", 1000, query_velocity_callback);

  ros::Subscriber query_position_sub = n.subscribe("query_position", 1000, query_position_callback);

  wheels_power_pub = n.advertise<base_controller::wheels_msg>("set_wheels_power", 1000);
  
  //_targetVel = 0.0001f;

  test_convert_twist_to_wheels_velocity();

  while(ros::ok())
  {

    updateWheelPower();
    ros::Duration(0.26).sleep();

    ros::spinOnce();
  }

  return 0;
}

