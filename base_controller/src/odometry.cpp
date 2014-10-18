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
 * @section DESCRIPTION
 * Odometry node that constantly track how far wheels have turned and use that to estimate current position. It has to subscribe to the query velocity and position topics, as well as compass heading.
 * Should publish odometry data and tf stuff as well.
 */

/*
 * based on code taken from: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
 */

/*
 * Since we're going to be publishing both a transfrom from the "odom" coordinate frame to the "base_link" coordinate frame and a nav_msgs
 * Odometry message, we need to include the relevant header files.
 */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

/*
 * We'll assume that the robot starts at the origin of the "odom" coordinate frame initially.
 */
  double x = 0.0; /// current x position in the odometry frame (in meters)
  double y = 0.0; /// current y position in the odometry frame (in meters)
  double th = 0.0; /// th is just theta (the current compass heading in the odometry frame)

  ros::Time current_time, last_time;

/** \brief takes query_position data (in ticks) and converts that to distances (in meters) (still a wheels_msg object)
 * \param query_position given query_position to be converted to meters
 * \return distance the wheels moved in meters (instead of ticks) as a wheels_msg object
 * 
 * converts query_position ticks data to distance travelled in meters using this formula:
 * distance_travelled = wheel_circumference * position_ticks / ticks_per_revolution;
 *
 * @todo grab Scipio dimensions from ROS parameters instead of hard-coded values!
 */
base_controller::wheels_msg convert_query_position_to_distance(base_controller::wheels_msg query_position)
{

}

/** \brief sets current compass heading to the value in the msg we just received.
 *
 */
void compass_heading_callback(compass::compass_heading_msg heading_msg)
{
  th = heading_msg.heading;
}
/** \brief uses query_position and compass_heading data to calculate distance travelled (in x and y direction) since last time and increments x and y values.
 * \param position_msg the wheels_msg containing position ticks data (ticks since last called)
 *
 * This method is called whenever new position ticks data arrives. We should grab position ticks and convert them to distances in meters, then use formula (using both wheel distances and also compass heading) to calculate new delta_x and delta_y, then increment these to x and y for this odometry frame.
 * Formula: delta_x = (left_distance + right_distance) * cos(heading) / 2;
 *          delta_y = (left_distance + right_distance) * sin(heading) / 2;
 *
 * Look at this deimos code in Java for tips:
    //TODO convert heading from compass-space to normal (counter-clockwise) space.
        theta_heading = 360.0 - theta_heading + 90.0;
        

        //using the compass heading, get the sine and cosine in radians
        double cosine_current = Math.cos(Math.toRadians(theta_heading));
        double sine_current = Math.sin(Math.toRadians(theta_heading));

        double deltaX = WHEEL_DRIVE_RADIUS * cosine_current * (leftPositionTicks + rightPositionTicks) * (Math.PI / TICKS_FULL_ROTATION);
        double deltaY = WHEEL_DRIVE_RADIUS * sine_current * (leftPositionTicks + rightPositionTicks) * (Math.PI / TICKS_FULL_ROTATION);
        
        // new position is computed
        currentX += deltaX;
        currentY += deltaY;
 */
void query_position_callback(base_controller::wheels_msg position_msg)
{
  /*
 * Here we'll set some velocities that will cause the "base_link" frame to move in the "odom" frame at a rate of 0.1m/s in the x direction, -0.1m/s in the y direction, and 0.1rad/s in the th direction. This will more or less cause our fake robot to drive in a circle.
 */

 /// @todo get the actual query position (convert it to meters)
 /// this is where you actually query position and calculate these values (instead of fake constants like now)
  // double vx = 0.1;
  // double vy = -0.1;
  // double vth = 0.1;

  
  current_time = ros::Time::now();

   
/*
 * Here we'll update our odometry information based on the constant velocities we set. A real odometry system would, of course, integrate computed velocities instead.
 */
 ///@todo change these to use position ticks instead of velocities

    //compute odometry in a typical way given the velocities of the robot
    // double dt = (current_time - last_time).toSec();
    // double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    // double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    // double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;

/*
 * We generally try to use 3D versions of all messages in our system to allow 2D and 3D components to work together when appropriate, and to keep the number of messages we have to create to a minimum. As such, it is necessary to convert our yaw value for odometry into a Quaternion to send over the wire. Fortunately, tf provides functions allowing easy creation of Quaternions from yaw values and easy access to yaw values from Quaternions.
 */
    // th += delta_th;
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

/*
 * Here we'll create a TransformedStamped message that we will send out over tf. We want to publish the transform from the "odom" frame to the "base_link" frame at current_time. Therefore, we'll set the header of the message and the child_frame_id accordingly, making sure to use "odom" as the parent coordinate frame and "base_link" as the child coordinate frame.
 */
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

/*
 * Here we fill in the transform message from our odometry data, and then send the transform using our TransformBroadcaster.
 */
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

/*
 * We also need to publish a nav_msgs/Odometry message so that the navigation stack can get velocity information from it. We'll set the header of the message to the current_time and the "odom" coordinate frame.
 */
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

/*
 * This populates the message with odometry data and sends it out over the wire. We'll set the child_frame_id of the message to be the "base_link" frame since that's the coordinate frame we're sending our velocity information in.
 */
    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;

}

int main(int argc, char** argv){

  ros::init(argc, argv, "odometry");

  ros::NodeHandle n;


  ros::Subscriber query_position_sub = n.subscribe("query_position", 1000, query_position_callback);

  ros::Subscriber compass_heading_sub = n.subscribe("compass_heading", 1000, compass_heading_callback);

/*
 * We need to create both a ros::Publisher and a tf::TransformBroadcaster to be able to send messages out using ROS and tf respectively.
 */
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  last_time = ros::Time::now();

  while(n.ok())
  {
    ros::spin();
  }


}
