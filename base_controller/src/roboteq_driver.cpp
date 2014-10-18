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
 * Driver node for Roboteq motor controllers. Uses Anup's SerialPort drivers to talk to the Roboteq motor controllers
 * and send commonly used commands (setting and getting stuff). This node subscribes to cmd_vel topic (takes in command
 * velocity) and tells Roboteqs to set wheel velocity accordingly. It also
 * regularly polls Roboteqs for wheel velocity and publishes them (as a twist) to current_velocity topic.
 * @todo need to use ROS parameters to set things like sampling time interval 
 * @todo consider createTimer instead of sleep.
 */
 
/// SerialPort includes
#include <iostream>
#include <sstream>
#include "./serialDriver/SerialPort.h"
#include <cstring>
#include <unistd.h>
#include "./serialDriver/OutputConsole.h"

/// ROS includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>        // Twist message file
#include <base_controller/wheels_msg.h> // wheels_msg message file
#include <base_controller/diagnostics_msg.h> // diagnostics_msg file
#include <string>


#define SAMPLING_RATE           5
#define BAUD_RATE               115200
/// Scipio dimensions and stuff
#define METERS_PER_TICK_SCIPIO  0.0011169116
#define WHEEL_DIAMETER_SCIPIO   0.3556
#define TICK_COLLECTION_PERIOD  0.05
#define TRACK_WIDTH             34
#define WHEEL_BASE              21
#define SLEEP_INTERVAL          0.05
#define RPM_TO_RAD_PER_SEC      0.1047

/// declare port object
SerialPort *roboteq_port_  =    NULL;

/// current velocity publisher object is declared
/// diagnostics publisher object is declared
/// @todo check if we need to free memory for this 
ros::Publisher current_velocity_pub_;
ros::Publisher diagnostics_pub_;
ros::Subscriber cmd_vel_sub;
/// current wheel velocity (of left and right wheels in rad/s)
/// diagnostics message object created
base_controller::wheels_msg wheel_velocity_;
base_controller::diagnostics_msg diagnostics_;

int initialized_dirtyBit = 0; // Only for test purposes. Once joystick can send commands rewrite this.

/** \brief sends command to roboteq and returns response as a string
 *  \param command the command to send to roboteq (as a string)
 *  \return a string containing the response received from the roboteq
 *  this method sends the given command to the roboteq and returns the response from the roboteq as a string.
 *  note that the returned string will contain extra garbage data at the end
 */
std::string sendCommandToRoboteq(std::string command_string)
{
  //const char command[] = "?S\r\0"; //don't forget the null character bit 
  char *command = new char [command_string.length()+1];
  std::strcpy (command, command_string.c_str());
  const int command_length = command_string.length() + 1;//sizeof(command);
  ROS_INFO("command length is: %d", command_length);

  roboteq_port_->write(command, command_length);
  
  ROS_INFO("written to port...");

  // wait a little bit before you read response.  
  ros::Duration(SLEEP_INTERVAL).sleep();

  // sample response: "?S\r\nS=0000:0000\r\n"
  int response_length = command_length + 34;
  unsigned char response[response_length];

  int read = roboteq_port_->read(response, response_length);  

  ROS_INFO("read from port...");

//if no response or unsuccessful command response (minus), return false
  if(read < 1)
  { 
    ROS_ERROR_STREAM("Failed to read from port after sending: " << command_string); 
  }

  delete command;
  // create a string out of the response (was a char*)
  std::string response_string(response, response + response_length);
  return response_string;
}


/** \brief checks whether given serial port is the Roboteq.
 *  \return 1 if serial port is Roboteq. Return 0 otherwise.
 *  Sends verification command to serial port and sees if response indicates it's a Roboteq.
 *  @todo check if serial port argument should be pointer or not 
 */
unsigned int checkConnection()
{
  std::string command_string = "?FID\r\0"; //don't forget the null character bit
  std::string correct_response_string = "Roboteq\0"; //don't forget the null character bit  
  try
  {

    roboteq_port_ = new SerialPort("/dev/ttyACM0",BAUD_RATE);

    //send firmware ID command to port
    std::string response_string = sendCommandToRoboteq(command_string);

    //read response and check it for "Roboteq" response fragment
    std::size_t found = response_string.find(correct_response_string);
  
    if (found!=std::string::npos)
    {
      ROS_INFO("port is roboteq_port_! \n%s", response_string.c_str());
      return true;
    }
    else
    {
      ROS_INFO("port is not roboteq_port_ or connection failed! \n%s",response_string.c_str());
      return 0;
    }
  }
  catch (SerialPortException& e)
  {
    ROS_ERROR_STREAM("" << e.what());
    return 0;
  }
}


/** \brief connects to Roboteq motor controller through serial port. Returns 1 on success, and 0 on failure.
 *  \return 1 on success; 0 on failure.
 *  Opens every serial port (one by one) and verifies if it's the Roboteq. If it finds it, it connects to it and returns 1.
 *  If it doesn't find it, it returns 0.
 */
 /* probably will be depreciated
unsigned int establishConnection()
{
  // loop through every serial port (up to /dev/ttyUSB10)
    //open serial port, and call checkConnection on it. 
    //If it is the Roboteq, return 1. Return 0 otherwise.
  ROS_DEBUG("Attempting to connect to Roboteq...");

  //connect to ports
  roboteq_port_ = NULL; //set it to null so it doesn't coredump when it reaches delete (and unassigned)
  // iterate through all ttyUSBx ports and find ROBOTEQ and set it
  //loop through 10 ports
  for(int i = 0; i < 10; i++)
  {

    std::string port_name;
    port_name = ("/dev/ttyUSB");
    port_name += char(i + 48); // i + 48 turns it into ascii, append that to portName
    ROS_DEBUG("trying port_name /dev/ttyUSB%d", i);

    //open port, send it identification command and wait for response (give it 100 ms)
            
    try
    {

      roboteq_port_ = new SerialPort(port_name, BAUD_RATE);
                
      ROS_DEBUG("Connected to /dev/ttyUSB%d", i);

      //call checkConnection
      unsigned int response = checkConnection();
      if(response > 0)
      {

        ROS_INFO("Connected to Roboteq at: /dev/ttyUSB%d", i);
        return 1; // if roboteq found, return 1 indicating success (no need to do other stuff in method)
      }
      else
      {
        ROS_DEBUG("Heard nothing from /dev/ttyUSB%d but I don't recognize it as Roboteq", i);
        delete roboteq_port_; 
        roboteq_port_ = NULL;     
      }

      //if we don't get a response, we just move on (and continue to beginning of for loop)

    }
    catch (SerialPortException& e)
    {
      ROS_DEBUG("Failed to connect to /dev/ttyUSB%d, so it's probably not Roboteq, moving on...", i);
    }
  } //end for loop (to iterate through /dev/ttyUSBx)

  //check that roboteq_port is there
  if(!roboteq_port_)
  {
    ROS_FATAL("Could not find Roboteq port!!!");
    return false;
  }
  //if Roboteq not found, return 0. 
  return 0;
}
*/

/** \brief converts wheel velocities (from RPM) to a twist (of the entire robot).
 *  \parameter left_velocity left wheel velocity in RPM
 *  \parameter right_velocity right wheel velocity in RPM
 *  \return a twist giving the velocity of the entire robot (linear and rotational).
 *  Takes in both wheel velocities, and returns the velocity of the entire robot as a twist.
 *  Uses dimensions of the robot (Scipio). Assumes negligible skid when turning.
 *  @todo check if this works on a skid-steer (or make sure robot only turns in place).
 *  @todo change param to a single wheels_msg object instead of two floats
 */
geometry_msgs::Twist convertWheelVelocityToTwist(float left_velocity, float right_velocity)
{

 	float longitudinal_velocity;
	
	geometry_msgs::Twist twist_velocity;

	longitudinal_velocity = (right_velocity - left_velocity)/2;

	twist_velocity.linear.x = right_velocity - longitudinal_velocity;

	twist_velocity.angular.z = (longitudinal_velocity * 2 * TRACK_WIDTH) / (TRACK_WIDTH ^ 2 + WHEEL_BASE ^ 2);

	return twist_velocity;

  //take a look at convertTwistToWheelVelocity in deprecated motor_controller

}

/** \brief converts a twist to corresponding wheel velocities (left and right) 
 *  \param twist_velocity target velocity given as a twist object.
 *  \return pair of float target velocities (first is left wheel and second is right wheel); a wheels_msg object
 *  takes in a target velocity as a twist, then converts it to velocity for wheels (as a wheels_msg object).
 *  First, the angular z velocity is converted to longitudinal velocity (with opposite sides having opposite values) using the formula:
 *  longitudinal_velocity = rotational_velocity * (track_width ^ 2  + wheel_base ^ 2) / 2;
 *  Then, the wheel_velocity = longitudinal_velocity + linear_x_velocity;
 */
base_controller::wheels_msg convertTwistToWheelVelocity(const geometry_msgs::Twist::ConstPtr& twist_velocity) // look into removing permanently
{
	float longitudinal_velocity;

	base_controller::wheels_msg wheel_velocity;

	longitudinal_velocity = ((twist_velocity->angular.z) * (TRACK_WIDTH ^ 2 + WHEEL_BASE ^ 2)) / (2 * TRACK_WIDTH);
  
    wheel_velocity.left = -1*longitudinal_velocity + twist_velocity->linear.x;
    wheel_velocity.right = longitudinal_velocity + twist_velocity->linear.x;


	return wheel_velocity;

  //take a look at convertTwistToWheelVelocity in deprecated motor_controller

}

/** \brief sends wheel velocity command to Roboteq motor controller
 *  \param left_velocity the velocity of the left wheels (in rad/s)
 *  \param right_velocity the velocity of the right wheels (in rad/s)
 *  
 *  takes in left and right wheel velocity (in rad/s) and commands the Roboteq motor controller (dual channel) to go that fast.
 *  First, the wheel velocities are converted from rad/s to RPM, then rounded off to an int. Then the ints are thrown into a string stream
 *  then converted to a string. The string command is sent to the roboteq, and the response is parsed to check for failure or success.
 */
void setWheelVelocity(base_controller::wheels_msg wheel_velocity)
{
    // for DEBUG only
  /*
  roboteq_port_ = new SerialPort("/dev/ttyUSB0", BAUD_RATE);
  */

  // @todo check if RPM values have decimal resolution
  // left and right wheel velocity in RPM (as a float)
  float left_velocity_RPM = wheel_velocity.left / RPM_TO_RAD_PER_SEC;
  float right_velocity_RPM = wheel_velocity.right / RPM_TO_RAD_PER_SEC;
  
  // now round the wheel velocity to int
  int left_velocity;
  if (left_velocity_RPM >= 0)
    left_velocity = (int) (left_velocity_RPM + 0.5); 
  else 
    left_velocity = (int) (left_velocity_RPM - 0.5);

  int right_velocity;
  if (right_velocity_RPM >= 0)
    right_velocity = (int) (right_velocity_RPM + 0.5); 
  else 
    right_velocity = (int) (right_velocity_RPM - 0.5);


  //now that we have our left and right velocities (in RPM), construct the command to send to roboteq
  std::stringstream ss; //make a string stream and then convert it to string
  std::stringstream sd;
  //@todo uncomment this when we get dual-channel roboteq
  //ss << "!G " << left_velocity*100 << ":"<< right_velocity*100 <<"\r";
  ss << "!G 1 " << left_velocity*100 << "\r";
  std::string command_string_1 = ss.str();
  sd << "!G 2 " << right_velocity*100 << "\r";
  std::string command_string_2 = sd.str();
  ROS_INFO_STREAM("305 Command String: " << command_string_1);
  ROS_INFO_STREAM("305 Command String: " << command_string_2);
  //send command to roboteq and get response
  std::string response_string_1 = sendCommandToRoboteq(command_string_1);
  std::string response_string_2 = sendCommandToRoboteq(command_string_2);



  //parse the response for tokens
  //std::string echo_string_1 = response_string_1.substr(0, response_string_1.find("\r"));
  //ROS_INFO_STREAM("echo string was: " << echo_string_1);
  std::string echo_string_2 = response_string_2.substr(0, response_string_2.find("\r"));
  ROS_INFO_STREAM("echo string was: " << echo_string_2);

  // FIX THIS. CONSTANTLY COMING UP FOR NEGATIVE INPUT

  //check if response contains a minus sign (and return it if it is)
  //std::size_t found = response_string.find("-");
  //if(found!=std::string::npos) //if we find minus sign in response_string
  //{
  //  ROS_ERROR_STREAM("Found a minus sign at index: " << found);
    //@todo return or something here
  //}

  //check if response contains a plus sign (and return it if it is)
  /*
  found = response_string.find("+");
  if(found!=std::string::npos) //if we find minus sign in response_string
  {
    ROS_INFO_STREAM("Command successful! Received a plus sign!");
    //@todo return or something here
  }else
  {
    ROS_ERROR_STREAM("Command failed! Did not receive a plus sign!");
    //@todo we have a problem here
  }
  */
/*
  delete roboteq_port_;
  ROS_INFO("Deleted port from memory");
  */
}


/** \brief queries wheel velocity from Roboteq motor controller
 *  \return a wheels_msg object with left and right wheel velocities (in rad/s)
 *  This method queries the Roboteq (dual channel) for both wheel velocity (in RPM) and returns them.
 */
base_controller::wheels_msg getWheelVelocity()
{
  // for debug only
  /*
  roboteq_port_ = new SerialPort("/dev/ttyUSB0", BAUD_RATE);
  ROS_INFO("port opened...");
  */
  

  std::string command_string = "?S\r\0"; //don't forget the null character bit 

  //send command to roboteq and get response
  std::string response_string = sendCommandToRoboteq(command_string);

  //parse the response for tokens
  std::string echo_string = response_string.substr(0, response_string.find("\r"));
  ROS_INFO_STREAM("echo string was: " << echo_string);

  //check if response contains a minus sign (and return it if it is)
  std::size_t found = response_string.find("-");
  if(found!=std::string::npos) //if we find minus sign in response_string
  {
    ROS_ERROR_STREAM("Found a minus sign at index: " << found);
    //@todo return or something here
  }

  // look for colon character (to check if this is a proper dual-channel output)
  found = response_string.find(":");
  if(found==std::string::npos) //if we don't find a colon, freak out!
  {
    ROS_ERROR("Not a dual-channel response! Did not find colon!");
    //@todo return or something here
  }

  // the values are between the first equals sign and second carriage return (watch for correct indexing)
  std::string value_string = response_string.substr(response_string.find("=")+1, response_string.find("\r", 2)+1);
  ROS_INFO_STREAM("value string was: " << value_string << " with length: " << value_string.length());

  // now get left value. Extracts the substring in value_string (from start till colon) and converts to int.
  // @todo check if RPM values have decimal resolution
  // @todo uncomment this when we get the dual-channel unit
  int left_rpm = atoi(value_string.substr(0, value_string.find(":")).c_str() );
  // get right value. Extracts substring in value_string (from after colon till end) and converts to int.
  int right_rpm = atoi( value_string.substr(value_string.find(":")+1, value_string.length()-1).c_str() );
  
  //create a new wheel velocity object to store the data we just pulled into
  base_controller::wheels_msg wheel_velocity;

  //store left and right wheel velocity in them (after converting from RPM to rad/s)
  //@todo uncomment this when we get the dual-channel unit
  wheel_velocity.left = left_rpm * RPM_TO_RAD_PER_SEC;
  wheel_velocity.right = right_rpm * RPM_TO_RAD_PER_SEC;

//goes through response char array and checks for return carriage
/*  for(int i=0; i<response_length; i++)
  {
    if(response[i] == 13)
      ROS_INFO("FOUND IT: %d %d", i, response[i]);
    else
      ROS_INFO("naaa... %d %d %c", i, response[i], response[i]);
  }*/
    /*
  //for DEBUG
  delete roboteq_port_;
  ROS_INFO("Deleted port from memory");
  */

  //@todo comment this out when you get the dual-channel unit
  //wheel_velocity.left = atoi(value_string.c_str());
  //return both left and right wheel velocity as rad/s
  return wheel_velocity;
}

/** \brief publishes current wheel velocity
 *  \param wheel velocity object
 */
void publishCurrentVelocity(geometry_msgs::Twist twist_velocity)
{
	current_velocity_pub_.publish(twist_velocity);
}

/** \brief publishes diagnostics
 *  \param diagnostics object
 */
void publishDiagnostics(base_controller::diagnostics_msg diagnostics)
{
  diagnostics_pub_.publish(diagnostics);
}

static int temp = 0;

/** \brief reads constant incoming messages from roboteq controller
 *  \returns false if reading failed
 *  reads the incoming messages from roboteq controller and, 
 *  depending on response (velocity or diagnostics), parses message and/or publishes message to a topic
 */
 unsigned int readFromRoboteq()
 {
  int diagnostics_published = 0;
  int velocity_published = 0;

  if (temp == 0)
  {
    std::string tele_string = "# C\r?S\r?A\r?M\r?FF\r?FS\r# 100\r\0";
    //std::string tele_string = "# C\r?S\r?A\r?M\r?FF\r?FS\r!G 1 500\r!G 2 500\r# 300\r\0"; //telemetry string, each response has a 200ms delay
    char *command = new char [tele_string.length()+1];
    std::strcpy (command, tele_string.c_str());
    const int command_length = tele_string.length() + 1;//sizeof(command);

    roboteq_port_->write(command, command_length);

    temp++;
  }

  int response_length = 16; // some response length that will be long enough to get all the data
  unsigned char response[response_length];

  //create a new diagnostics object to store data
  base_controller::diagnostics_msg diagnostics_;

  // while both messages aren't published, keep reading
  while(!velocity_published || !diagnostics_published) 
  {

    int read = -1;
    while((read = roboteq_port_->read(response, response_length)) == -1);
    std::string response_string(response, response + response_length); //convert to std::string

    if(read<1)
    {
      ROS_ERROR("Failed to read from port");
      //return 0; 
    } 
    else
    {
      std::size_t wheel_velocity = response_string.find("S");
      std::size_t status_flags = response_string.find("FS");
      std::size_t fault_flags = response_string.find("FF");
      std::size_t amperage = response_string.find("A");
      std::size_t motor_power = response_string.find("M");

      if(status_flags!=std::string::npos) // if response is status flags, populate diagnostics_.status_flags and publish (last message in message loop)
      {

        diagnostics_.status_flags = response_string;
        publishDiagnostics(diagnostics_);
        diagnostics_published = 1;
      }
      else if(wheel_velocity!=std::string::npos)// && status_flags==std::string::npos) //if response is wheel velocity parse and publish to current_vel topic
        // have to add && condition because status flags and wheel velocity share S= charaters
      {
        // the values are between the equals sign and carriage return (watch for correct indexing)
        std::string value_string = response_string.substr(response_string.find("=")+1, response_string.find("\r", 2)+1);
        ROS_INFO_STREAM("value string was: " << value_string << "\nwith length: " << value_string.length());

        // now get left value. Extracts the substring in value_string (from start till colon) and converts to int.
        // @todo check if RPM values have decimal resolution
        // @todo uncomment this when we get the dual-channel unit
        int left_rpm = atoi(value_string.substr(0, value_string.find(":")).c_str() );
        // get right value. Extracts substring in value_string (from after colon till end) and converts to int.
        int right_rpm = atoi( value_string.substr(value_string.find(":")+1, value_string.length()-1).c_str() );
    
        //create a new wheel velocity object to store the data we just pulled into
        base_controller::wheels_msg wheel_velocity_;

        //store left and right wheel velocity in them (after converting from RPM to rad/s)
        //@todo uncomment this when we get the dual-channel unit
        wheel_velocity_.left = left_rpm * RPM_TO_RAD_PER_SEC;
        wheel_velocity_.right = right_rpm * RPM_TO_RAD_PER_SEC;

        //convert to twist and publish wheel velocity to current_velocity topic
        publishCurrentVelocity(convertWheelVelocityToTwist(wheel_velocity_.left,wheel_velocity_.right));
        velocity_published = 1;
      }
      if(amperage!=std::string::npos) // if response is amperage, populate diagnostics_.amperage
      {
        diagnostics_.amperage = response_string;
      }
      if(motor_power!=std::string::npos) // if response is motor power, populate diagnostics_.motor_power
      {
        diagnostics_.motor_power = response_string;
      }
      if(fault_flags!=std::string::npos) // if response is fault flags, populate diagnostics_.fault_flags
      {
        diagnostics_.fault_flags = response_string;
      }
    }
    ros::Duration(0.1).sleep(); // sleep until next message from roboteq arrives
  }
}

/** \brief sends set wheel velocity to Roboteq when we receive a message on cmd_vel
 *  \param twist_velocity a velocity command Twist (pair of vectors; one is linear, the other is rotational)
 *
 *  Called when this node receives a message on cmd_vel topic. Sends a set wheel velocity command to the Roboteq motor controller. 
 */
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& twist_velocity)
{
  base_controller::wheels_msg wheel_velocity = convertTwistToWheelVelocity(twist_velocity);
  setWheelVelocity(wheel_velocity);
}


/** \brief starts roboteq_driver and all its connections
 *
 * 	Starts the node, then subscribes to topic (cmd_vel) and advertises topic. Then, it connects to Roboteq and queries current velocity then publishes it (after converting it to a twist).
 *  Checks connection before every loop. If check fails (or connection fails), it waits 3 seconds then kills itself (should be restarted by launch file).
 */
int main(int argc, char **argv)
{
  
    
  // initialize ROS node named roboteq_driver  
  ros::init(argc, argv, "roboteq_driver");

   

    

  //ROS_INFO("roboteq_driver node started...");

  //create handle for node (used to advertise topics)
  ros::NodeHandle n;

  if(checkConnection())
  {

    // create subscriber object and have it subscribe to topic named cmd_vel with a callback function named cmdVelCallback
    

    // create publisher object and advertise to topic named current_velocity with message queue of 1. The messages we send will be
    // of type twist, which is basically a pair of vectors (one linear, and the other rotational). (no longer true)
    // also creates status flag publisher to publish status flags to topic named diagnostics, message will be of type unsigned int
    /// @todo check message queue size here 
    current_velocity_pub_ = n.advertise<geometry_msgs::Twist>("current_velocity", 1);
    diagnostics_pub_      = n.advertise<base_controller::diagnostics_msg>("diagnostics",1);
    cmd_vel_sub           = n.subscribe("cmd_vel", 1, cmdVelCallback);


    //keep trying to establish connection. Leave while loop when you succeed.
    /// @todo consider killing node if establishConnection fails (since roslaunch restarts it automatically). Keep behavior consistent with checkConnection.
    while(ros::ok())
    {
      
      readFromRoboteq(); // once per loop read from the roboteq driver and populate and publish wheel velocity and fault flags to respective topics
      /* Test of setWheelVelocity()
      base_controller::wheels_msg wheel_velocity_;
      wheel_velocity_.left = 0.1; // Tests
      wheel_velocity_.right = 0.1; //Tests
      setWheelVelocity(wheel_velocity_); // Tests
      */
      //get wheel velocity from roboteq
      //wheel_velocity_ = getWheelVelocity();
      // publish it to current_vel topic
      //publishCurrentVelocity(convertWheelVelocityToTwist(wheel_velocity_.left, wheel_velocity_.right));
      ros::spinOnce(); // check for new messages on subscribed topics
    }

   





  /// make sure to delete port objects from memory (never actually going to reach here)
  //delete roboteq_port_;

  /////// TESTS and stuff /////////
  /*int x = 0;

   while(ros::ok())
    {
    
    geometry_msgs::Twist My_Twist;
    My_Twist.linear.x = x;
    publishCurrentVelocity(My_Twist);
    x++;
    }*/
    
    /*
    if (initialized_dirtyBit == 0)
    {
      base_controller::wheels_msg a_message = getWheelVelocity();
      ROS_INFO("queried wheel velocity and got: [%f %f]", a_message.left, a_message.right);
      initialized_dirtyBit = 1;
    }
    */

    
    
    //a_message.left++;
    //a_message.right++;
    //setWheelVelocity(a_message);

    //working state    

    ros::spin();

    delete roboteq_port_;
    return 0;
  }

}
