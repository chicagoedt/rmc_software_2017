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
 * Test node for edcs_driver. Simply starts itself up and sends over wheels_msg messages over the set_wheels_power topic. Attempts to
 * ramp the motor up then down. @todo also need to test query velocity and query position ticks.
 */
 
/// ROS includes
#include "ros/ros.h"
#include <base_controller/wheels_msg.h>  ///wheel power msg

using namespace std;

ros::Publisher pub_set_wheels;

/** \brief runs a short test by sending wheels_msg messages to edcs_driver
 *	
 *	Starts wheel power at 8%, then ramps up to 34%, then ramps down to 0%.
 */
void test_wheels(){
    base_controller::wheels_msg power_msg;
    int i=0;
    int sign = -1;
    ros::Rate loop_rate(2);
    int count = 0;
    int max_ramp = 30; ///ramp up to this maximum power
    int sleep_duration = 15;
    while(ros::ok() && count< max_ramp*4 + 1){

        if(i == max_ramp || i ==-max_ramp)
        {   
            ros::Duration(sleep_duration).sleep(); 
            sign = -sign;
        }

        power_msg.left= i*0.01;
        power_msg.right = i*0.01;
        ROS_DEBUG("Setting test wheel power to: [%f %f]", power_msg.left, power_msg.right);
        pub_set_wheels.publish(power_msg);
        
        i = i + sign;
        loop_rate.sleep();
        count++;
    }

    power_msg.left = 0;
    power_msg.right = 0;

    pub_set_wheels.publish(power_msg);

}

/** \brief starts edcs_driver_test and all its connections
 *
 * 	Starts the node, then advertises topic (set_wheels_power). Then, it runs a test for EDCS. 
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "edcs_driver_test");

  ros::NodeHandle n;

  pub_set_wheels = n.advertise<base_controller::wheels_msg>("set_wheels_power", 1000);
  
  test_wheels();

  ROS_INFO("edcs_driver_test done! Shutting down!");

  return 0;
}

