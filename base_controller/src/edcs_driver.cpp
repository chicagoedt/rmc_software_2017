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
 * Driver node for EDCS (in Scipio robot). Uses Anup's SerialPort drivers. Use this node to communicate
 * with the EDCS, and do things like querying for wheel ticks or setting motor power levels. This node  
 * subscribes to a set_wheels_power topic, and publishes query velocity and position data.
 * @todo need to use ROS parameters to set things like sampling time interval 
 */
 
/// SerialPort includes
#include <iostream>
#include "./serialDriver/SerialPort.h"
#include <cstring>
#include <unistd.h>
#include "./serialDriver/OutputConsole.h"

/// ROS includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <base_controller/wheels_msg.h> /// query velocity message file
#include <string>


/// EDCS command definition codes
#define TEST_CONNECTION         0xFF
#define CONNECTION_CORRECT      0x55
#define COMMAND_SUCCESSFUL      0xC0
#define MAX_POWER               0x7D
#define SYSTEM_IDENTIFICATION   0xC9
#define LEFT_SIDE_EDCS          0x16
#define RIGHT_SIDE_EDCS         0x17
#define SET_MOTOR_POWER         0xD3
#define VELOCITY_TICKS_QUERY    0xC5
#define POSITION_TICKS_QUERY    0xC3
#define SLEEP_INTERVAL          0.01
#define SAMPLING_RATE           5
/// Scipio dimensions and stuff
#define METERS_PER_TICK_SCIPIO  0.0011169116
#define WHEEL_DIAMETER_SCIPIO   0.3556
#define TICK_COLLECTION_PERIOD  0.05

/// declare port objects for left and right EDCS
 SerialPort *leftPort = NULL;
 SerialPort *rightPort = NULL;

/// query velocity publisher
 ros::Publisher query_velocity_pub;
/// query position publisher
 ros::Publisher query_position_pub;

 using namespace std;

/** \brief converts query position response to int
 *  \param weird_position_bits[] the response bits from query position
 *  \return converted int
 *  
 *  converts weird 28-bit position query to an int
 *  note that the uppermost bit in every data byte is just a flag bit
 *  data bytes are sent from EDCS most significant first!
 *  ignores first byte (status byte)
 */
 int convert_position_query_to_int(uint8_t weird_position_bits[])
 {
    uint32_t dataBytes[4];
    dataBytes[0] = weird_position_bits[1] & 0x7F; // zero out the highest bit (flag bit)
    dataBytes[1] = weird_position_bits[2] & 0x7F;
    dataBytes[2] = weird_position_bits[3] & 0x7F;
    dataBytes[3] = weird_position_bits[4] & 0x7F;

    uint32_t temp = (dataBytes[0] << 21) | (dataBytes[1] << 14) | (dataBytes[2] << 7) | (dataBytes[3]); 
    
    temp <<= 4; //shift logical to left (to put the sign bit in place)

    int32_t result = (int32_t)(temp) >> 4; // now shift arithmetic to the right (without fiddling with sign bit)

    return result;
}



/** \brief converts query velocity response to int
 *  \param weird_velocity_bits[] the response bits from query velocity
 *  \return converted int
 *  
 *  converts weird 14-bit velocity query to an int
 *  note that the uppermost bit in every data byte is just a flag bit
 *  data bytes are sent from EDCS most significant first!
 *  ignores first byte (status byte)
 */
 int convert_velocity_query_to_int(uint8_t weird_velocity_bits[])
 {
    uint16_t mostSignificantByte = weird_velocity_bits[1] & 0x7F;
    uint16_t leastSignificantByte = weird_velocity_bits[2] & 0x7F;
    uint16_t temp = (mostSignificantByte << 7) | leastSignificantByte;

    temp <<= 2; //shift logical to left (to put the sign bit in place)

    int16_t result = (int16_t)(temp) >> 2; // now shift arithmetic to the right (without fiddling with sign bit)
    
    return result; 
}


/** \brief converts power percentage power command
 *  \param power percentage of power to set motor to
 *  \param command[] the command byte to populate
 *  
 *  converts the int power value to a command that can be sent to EDCS
 *  takes in 3 int command array and fills it in for ya...
 */
 void prepare_set_power_command(uint8_t power, uint8_t command[])
 {
    command[0] = SET_MOTOR_POWER;
    command[1] = power & 0x7F;
    command[2] = ((power & 0x80)>>7) & 0x01;
}

/** \brief checks that both EDCS ports are connected
 *  \return true if connection is successful, false if not
 *  
 *  sends TEST_CONNECTION byte to each EDCS and checks response. Returns false if it doesn't work.
 */
 uint8_t check_connection()
 {

//for leftPort
    //send test_connection command
    char command[1];
    command[0] = TEST_CONNECTION;
    leftPort->write(&command[0], 1);

    //wait a while before reading
    ros::Duration(SLEEP_INTERVAL).sleep();

    //read response and check it
    char cc[2];
    int read = leftPort->read(&cc[0], 2);
    if (read > 0 && cc[1] == CONNECTION_CORRECT)
    {
        ROS_DEBUG("leftPort connection checked...! %x", cc[1]);
    }else
    {
        ROS_FATAL("leftPort connection failed! Response byte: %x", cc[1]);
        return false;
    }

//now do the exact same for rightPort
    //send test_connection command

    rightPort->write(&command[0], 1);

    //wait a while before reading
    ros::Duration(SLEEP_INTERVAL).sleep();

    //read response and check it
    memset(cc, 0, 2);
    read = rightPort->read(&cc[0], 2);
    if (read > 0 && cc[1] == CONNECTION_CORRECT)
    {
        ROS_DEBUG("rightPort connection checked...! %x", cc[1]);
    }else
    {
        ROS_FATAL("rightPort connection failed! Response byte: %x", cc[1]);
        return false;
    }

    return true;

}

/** \brief creates port objects and connects to EDCS
 *  \return false if failed to connect, true if connected
 *  
 *  Iterates through /dev/ttyUSBx and tries to connect and send SYSTEM_IDENTIFICATION command and waits for either LEFT_SIDE_EDCS or RIGHT_SIDE_EDCS response then sets ports to appropriate sides.
 *  Then calls check connection. Returns false if failed to connect to both EDCS.
 */
 uint8_t connect_EDCS()
 {

   ROS_DEBUG("Attempting to connect to EDCS...");
	//try{

        //connect to ports
        SerialPort *tempPort = NULL; //set it to null so it doesn't coredump when it reaches delete (and unassigned)
        // iterate through all ttyUSBx ports and find left and right EDCS's and set them
            //loop through 10 ports
        for(int i = 0; i < 10; i++)
        {

            std::string portName;
            portName = ("/dev/ttyUSB");
            portName += char(i + 48); // i + 48 turns it into ascii, append that to portName
            ROS_DEBUG("trying portName /dev/ttyUSB%d", i);

                //open port, send it identification command and wait for response (give it 100 ms)
            
            try{

                tempPort = new SerialPort(portName,  38400);
                
                ROS_DEBUG("Connected to /dev/ttyUSB%d", i);

                //send identification byte and wait for response
                uint8_t command[1]; command[0] = SYSTEM_IDENTIFICATION;
                tempPort->write(command, 1);

                ros::Duration(0.01).sleep(); //wait between writing command and reading response

                uint8_t response[2];
                uint8_t read = tempPort->read(response, 2);

                //if we get a response
                if(read > 0)
                {
                    // if left isn't already open and response is left side, then set this port to leftPort
                    if(!leftPort && response[1] == LEFT_SIDE_EDCS)
                    {
                        ROS_INFO("Connected to left EDCS at: /dev/ttyUSB%d", i);

                        leftPort = tempPort; 
                        tempPort = NULL;
                    // if right isn't already open and response is right side, then set this port to rightPort
                    }else if(!rightPort && response[1] == RIGHT_SIDE_EDCS)
                    {
                        ROS_INFO("Connected to right EDCS at: /dev/ttyUSB%d", i);
                        rightPort = tempPort;
                        tempPort = NULL;
                    }else
                    {
                        ROS_DEBUG("Heard [%x %x]from /dev/ttyUSB%d but I don't recognize it as EDCS", response[0], response[1], i);
                    }
                }//if we don't get a response, we just move on (and continue to beginning of for loop)

            }catch (SerialPortException& e){
                ROS_DEBUG("Failed to connect to /dev/ttyUSB%d, so it's probably not an EDCS, moving on...", i);
            }

            if(tempPort != NULL) //if tempPort is still set to something, make sure it's deleted
            {
                delete tempPort; //delete when done
                tempPort = NULL; // set it to NULL so it doesn't core dump on the next delete call
            }
        } //end for loop (to iterate through /dev/ttyUSBx)

        //check that both leftPort and rightPort are there
        if(!leftPort || !rightPort){
            ROS_FATAL("Could not find either left or right port!!!");
            return false;
        }
        
        ROS_DEBUG("now checking connection...");

        //check connection to ports by sending TEST_CONNECTION byte and read response
        if(!check_connection())
         return false;

     ROS_DEBUG("Succeeded at connecting to both left and right EDCS! YAY! :D");
     return true;
 }


/** \brief sets power level of both wheels
 *  \param leftPercentage percentage of power to set left motor to
 *  \param rightPercentage percentage of power to set right motor to
 *  
 *  @todo check float percentage conversion to int (maybe it should do rounding)
 *  Attempts to set motors to given power, and returns false if failed, true if successful.
 */
 uint8_t set_wheels_power(float leftPercentage, float rightPercentage)
 {

    //check if percentages are legal
   if(leftPercentage < -1.0f || leftPercentage > 1.0f){
      ROS_WARN("Attempted to set impossible left wheel power level: %f%%", leftPercentage*100);            
      return false;
  }

    //now figure out what we have to send to left EDCS
  uint8_t power = (uint8_t)(MAX_POWER * leftPercentage);
  uint8_t command[3]; prepare_set_power_command(power, command);

    //send command to left EDCS
  uint8_t response[1];

  leftPort->write(command, 3);
  ros::Duration(SLEEP_INTERVAL).sleep();

  int read = leftPort->read(response, 1);  

    if(read < 1 || !(response[0] & COMMAND_SUCCESSFUL) ) //if no response or unsuccessful response, return false
    {  
        ROS_ERROR("Failed to set left wheel power level with response %x", response[0]);
        return false;
    }

    //for debugging 
    ROS_DEBUG("set left wheel power to: %f converts it to command: [%x, %d, %d] with response: %x", leftPercentage, command[0], command[1], command[2], response[0]);

    //now figure out what we have to send to right EDCS
    if(rightPercentage < -1.0f || rightPercentage > 1.0f){
        ROS_WARN("Attempted to set impossible right wheel power level: %f%%", rightPercentage*100);            
        return false;
    }

    //send command to right EDCS
    power = (uint8_t)(MAX_POWER * rightPercentage);
    prepare_set_power_command(power, command);

    //send command to right EDCS and check its response
    rightPort->write(command, 3);
    
    ros::Duration(SLEEP_INTERVAL).sleep();

    read = rightPort->read(response, 1);
    
    if(read < 1 || !(response[0] & COMMAND_SUCCESSFUL) ) //if no response or unsuccessful response, return false
    {  
        ROS_ERROR("Failed to set right wheel power level with response %x", response[0]);
        return false;
    }

    //for debugging 
    ROS_DEBUG("set right wheel power to: %f converts it to command: [%x, %d, %d] with response: %x", leftPercentage, command[0], command[1], command[2], response[0]);

    return true;
}


/** \brief queries the EDCS for position ticks
 *  \param position_ticks[] array of two ints that will be populated with position ticks from EDCS
 *  \return true if successful and false otherwise
 *
 *  Writes query position ticks command to EDCS and reads response and checks it then converts it, then fills in position_ticks[]
 * 	parameter with it. 
 *	Hand it in positions array and it will fill it in with queried position ticks
 *	first element is left position and second element is right position
 */
 uint8_t query_position_wheels(int position_ticks[])
 {

    uint8_t commandByte[1]; commandByte[0] = POSITION_TICKS_QUERY;
    uint8_t response[5];

        // query left wheel position
    leftPort->write(commandByte, 1);
    
    ros::Duration(SLEEP_INTERVAL).sleep();

    uint8_t read = leftPort->read(response,5);

    if(read < 1 || !(response[0] & COMMAND_SUCCESSFUL) ) //if no response or unsuccessful response, return false
    {  
        ROS_ERROR("Failed to query left wheel position ticks with response: %x", response[0]);
        return false;
    }

    position_ticks[0] = convert_position_query_to_int(response); // convert response to actual position ticks

    ROS_DEBUG("left query position gives this response: %x %d %d %d %d", response[0], response[1], response[2], response[3], response[4]);

        // query right wheel position
    rightPort->write(commandByte, 1);
    
    ros::Duration(SLEEP_INTERVAL).sleep();

    read = rightPort->read(response, 5);

    if(read < 1 || !(response[0] & COMMAND_SUCCESSFUL) ) //if no response or unsuccessful response, return false
    {  
        ROS_ERROR("Failed to query right wheel position ticks with response: %x", response[0]);
        return false;
    }

    position_ticks[1] = convert_position_query_to_int(response); // convert response to actual position ticks

    ROS_DEBUG("right query position gives this response: %x %d %d %d %d", response[0], response[1], response[2], response[3], response[4]);
    return true;

}

/** \brief queries the EDCS for velocity
 *  \param velocities[] array of two ints that will be populated with velocities from EDCS
 *  \return true if successful and false otherwise
 *
 *  Writes query velocity command to EDCS and reads response and checks it then converts it, then fills in velocities[] parameter with  
 *	it. 
 *	Hand it in velocities array and it will fill it in with queried velocity
 *	first element is left velocity and second element is right velocity
 */
 uint8_t query_velocity_wheels(int velocities[])
 {

    //get fake query_velocity data and throw into converter and check that


    uint8_t commandByte[1]; commandByte[0] = VELOCITY_TICKS_QUERY;
    uint8_t response[3];

    // query left wheel velocity
    leftPort->write(commandByte, 1);
    
    ros::Duration(SLEEP_INTERVAL).sleep();

    int read = leftPort->read(response,3);

    if(read < 1 || !(response[0] & COMMAND_SUCCESSFUL) ) //if no response or unsuccessful response, return false
    {  
        ROS_ERROR("Failed to query left wheel velocity with response: %x", response[0]);
        return false;
    }

    ROS_DEBUG("left query velocity gives this response: %x %d %d", response[0], response[1], response[2]);

    velocities[0] = convert_velocity_query_to_int(response); //convert response to actual velocity

    // query right wheel velocity
    rightPort->write(commandByte, 1);
    
    ros::Duration(SLEEP_INTERVAL).sleep();

    read = rightPort->read(response,3);

    if(read < 1 || !(response[0] & COMMAND_SUCCESSFUL) ) //if no response or unsuccessful response, return false
    {  
        ROS_ERROR("Failed to query right wheel velocity with response: %x", response[0]);
        return false;
    }

    ROS_DEBUG("right query velocity gives this response: %x %d %d", response[0], response[1], response[2]);

    velocities[1] = convert_velocity_query_to_int(response); //convert response to actual velocity

    return true;    
}


/** \brief runs a short test for set_wheels_power and query commands
 *	
 * @deprecated the same test is now being run in a separate edcs_driver_test node
 *
 *	Starts wheel power at 8%, then ramps up to 34%, then ramps down to 0%. Prints query_velocity on each step and query_position 
 *	when it reaches 34% and when it finishes.
 */
// void test_wheels(){
//       int i=8;
//       int sign = 1;
//       while(i<35){

//         if(i>33)
//         {
//             sign = -sign;
//             int position_array[2] = {0,0};
//             query_position_wheels(position_array);
//             ROS_DEBUG("The left pos is: %d and the right is: %d", position_array[0],position_array[1]);
//         }
//         if(i==-1)
//             break;

//         set_wheels_power(i*0.01, i*0.01);
//         i = i + sign;
//         sleep(2);

//     int velocity_array[2] = {0,0};
//     query_velocity_wheels(velocity_array);
//     ROS_DEBUG("The left velocity is: %d and the right is: %d", velocity_array[0], velocity_array[1]);


//     // //DEBUGGING only. test convert_query_velocity manually:
//     // uint8_t test_velocity[3] = {0xC2, 85, 29};
//     // int result = convert_velocity_query_to_int(test_velocity);
//     // ROS_DEBUG("TESTING conv vel of [%x %d %d] to %d", test_velocity[0], test_velocity[1], test_velocity[2], result);

//     }
//     int position_array[2] = {0,0};
//     query_position_wheels(position_array);
//     ROS_DEBUG("The left pos is: %d and the right is: %d", position_array[0],position_array[1]);

//     set_wheels_power(0,0);

// }


/** \brief sets wheels power according to message it receives
 *  \param power_msg left and right wheel power (in percent)
 *
 *	sets wheel power based on power_msg.
 */
 void wheelsPowerCallback(const base_controller::wheels_msg power_msg)
 {
  ROS_DEBUG("I heard something from set_wheels_power topic!");
  ROS_DEBUG("Setting wheels to: [%f, %f]!", power_msg.left, power_msg.right);

      // set wheels power according to power_msg
  set_wheels_power(power_msg.left, power_msg.right);
}


/** \brief starts edcs_driver and all its connections
 *
 * 	Starts the node, then subscribes to topics (set_wheels_power). Then, it connects to EDCS and listens to topics and publishes query data. Checks connection before every loop.
 *  If check fails (or connection fails), it waits 3 seconds then kills itself (should be restarted by launch file).
 */
 int main(int argc, char **argv)
 {
  ros::init(argc, argv, "edcs_driver");

  ros::NodeHandle n;

  ros::Subscriber sub_set_wheels = n.subscribe("set_wheels_power", 1, wheelsPowerCallback);

  query_velocity_pub = n.advertise<base_controller::wheels_msg>("query_velocity", 1);

  query_position_pub = n.advertise<base_controller::wheels_msg>("query_position", 1);

  ros::Rate loop_rate(SAMPLING_RATE);

    // as long as ros runs, listen to commands
  while(ros::ok())
  { 
    //attempts to connect to EDCS, if fails, kills node
    if(connect_EDCS())
    {
        while(check_connection())
        {
            // read in query_velocity and publish

            int query_velocity[2];

            // actually query the velocity of the wheels and publish it (only if it worked)
            if(query_velocity_wheels(query_velocity))
            {        

            //now create a message for query velocity and fill it with the data we got from querying velocity
                base_controller::wheels_msg velocity_message;
                velocity_message.left = query_velocity[0];
                velocity_message.right = query_velocity[1];

            //ROS_INFO("query_velocity_wheels gave back [%d %d]", query_velocity[0], query_velocity[1]);
            // publish the message
                query_velocity_pub.publish(velocity_message);
            }

            // now read in query_position and publish
            int query_position[4];

            // actually query the velocity of the wheels and publish it (only if it worked)
            if(query_position_wheels(query_position))
            {        

            //now create a message for query velocity and fill it with the data we got from querying velocity
                base_controller::wheels_msg position_message;
                position_message.left = query_position[0];
                position_message.right = query_position[1];

                ROS_DEBUG("query_position_wheels gave back [%d %d]", query_position[0], query_position[1]);

            // publish the message
                query_position_pub.publish(position_message);
            }

            ros::spinOnce();   //check for messages on the topics we're subscribed to
            loop_rate.sleep(); // wait the loop_rate time interval
        } //end while check_connection()

    //if fail to connect_EDCS(), kill node
    } else
    {
        ROS_FATAL("Shutting down and restarting in 3 seconds...");
        ros::Duration(3.0).sleep();
            /// make sure to delete port objects from memory before killing node
        delete leftPort;
        delete rightPort;
        return 1;
    }


}      

/// make sure to delete port objects from memory (never actually going to reach here)
delete leftPort;
delete rightPort;

return 0;

}