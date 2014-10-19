/**		
 *Program:		encoder.cpp
 *Engineer(s):		John Sabino
 *Organization:		Engineering Design Team (University of Illinois at Chicago)
 *Date Created:		03/06/2014
 *
 *Last Data Modified:	05/17/2014
 *
 *Revision:
 *	0.1 - (03/06/2014) - File created.
 *	0.2 - (03/12/2014) - Improved the joystick algorithm.
 *	0.3 - (03/20/2014) - Removed the use of the Deadman's switch. Also added Linear + Mechanism button 
 *			     commands.
 *	0.4 - (03/27/2014) - Optimized the if statements for power level calculation. 
 *	0.5 - (05/17/2014) - Added a Manual E-GO command to correct a RoboteQ issue. 
 *			     Added an E-Stop command for emergencies.
 *******************************************************************/


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//Preprocessor Flags:
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define WiFi_Function_Enabled


////////////////////////////
//Preprocessor Definitions:
////////////////////////////

	//Wait time per command.
#define Sleep_Time     		1	//100	//ms

	//UDP Port definition:
#define PORT 				2390

	//Button Function Configurations:
#define Linear_Actuator_Up		8	// Y
#define Linear_Actuator_Home		2	// B
#define Linear_Actuator_Down		1	// A
//#define Turn_Mechanism_Motor_On		2	// B
#define Turn_Mechanism_Motor_On		64	// Right Trigger
#define Turn_Mechanism_Motor_Off	4	// X 
#define E_STOP				16	//Left Bumper
#define Manual_E_GO			32	//Right Bumper
#define Mechanism_Motor_Full_Reverse	128	//Back Button
//#define Deadman_Switch			16	// Left Bumper
#define Both_Wheels_Half_Forward	256	//Left Trigger

#define Mechanism_Speed			15	//7 	//

#ifdef WiFi_Function_Enabled
	#include <stdio.h>
	#include <sys/socket.h>
	#include <sys/types.h>
	#include <string.h>
	#include <unistd.h>
	#include <netinet/in.h>         //Needed for internet domain addresses (may not be necessary).
	#include <arpa/inet.h>
#endif

#include <stdlib.h>
#include <unistd.h>		//Needed to call the usleep command to sleep the application for so many uS.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <base_controller/Xbox_Button_Msg.h>
#include <iostream>


using namespace std;

//*********************************
//Function Prototypes:
//*********************************
void Button_Callback(const base_controller::Xbox_Button_Msg &Controller_Buttons);
void Joystick_Callback(const geometry_msgs::Twist &Controller_Joystick);
uint8_t Send_UDP (const unsigned char *data);
void ERROR (const char *msg);

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//Global Variables:
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static volatile uint16_t Flags = 0; 


int main (int argc, char** argv)
{
	ros::init(argc, argv, "Kevzak_Mission_Control_Encoder");
	ros::NodeHandle n;
	ros::Subscriber Button_Sub = n.subscribe("xbox_controller", 1000, &Button_Callback);
	ros::Subscriber Joystick_Sub = n.subscribe("cmd_vel", 1000, &Joystick_Callback);

	ros::spin();

	return 0;
}//End main

void Button_Callback(const base_controller::Xbox_Button_Msg &Controller_Buttons)
{
	Flags = 0;

	if (Controller_Buttons.a)			{Flags |= 1;}
	if (Controller_Buttons.b)			{Flags |= 2;}
        if (Controller_Buttons.x)			{Flags |= 4;}
        if (Controller_Buttons.y)			{Flags |= 8;}
        if (Controller_Buttons.left_bumper)		{Flags |= 16;}
        if (Controller_Buttons.right_bumper)		{Flags |= 32;}
	if (Controller_Buttons.right_trigger < 0)	{Flags |= 64;}
	if (Controller_Buttons.back_button)		{Flags |= 128;}
	if (Controller_Buttons.left_trigger < 0)	{Flags |= 256;}
/*
	ROS_INFO_STREAM( "A:  " << Controller_Buttons.a 	    << endl 
		      << "B:  " << Controller_Buttons.b 	    << endl 
 		      << "X:  " << Controller_Buttons.x 	    << endl 
		      << "Y:  " << Controller_Buttons.y 	    << endl 
		      << "LB: " << Controller_Buttons.left_bumper   << endl
		      << "RB: " << Controller_Buttons.right_bumper  << endl
		      << "BK: " << Controller_Buttons.back_button   << endl
		      << "ST: " << Controller_Buttons.start_button  << endl
		      << "PB: " << Controller_Buttons.power_button  << endl
		      << "LT: " << Controller_Buttons.left_trigger  << endl
		      << "RT: " << Controller_Buttons.right_trigger << endl
		      << "------------------------" << endl << endl << endl);
	
*/
}//End Button_Callback

/*Joystick_Callback*/
void Joystick_Callback(const geometry_msgs::Twist &Controller_Joystick)
{
	//Message Structure:
	//  8 bits:
	//	EOMMDPPP

	unsigned char Message = 64;




	//In the event that the Left Bumper is held, send the E-Stop command: 
	if (Flags & E_STOP)	
	{
		Message = 32;
                Send_UDP(&Message);
                return;
	}//End if

	switch(Flags)
	{
		case Manual_E_GO:
			Message |= 128;
			Send_UDP(&Message);
		break;
                
		case (Turn_Mechanism_Motor_On + Linear_Actuator_Down):
                        Message |= Mechanism_Speed;
                case (Turn_Mechanism_Motor_Off + Linear_Actuator_Down):
                        Message |= 96;
                        //Send the Mechanism Command:
                        Send_UDP(&Message);
                        //Wait for a 100 milliseconds before transmitting the next command.
                        usleep(Sleep_Time);
		case Linear_Actuator_Down:
			Message = 72;
		case Linear_Actuator_Up:
			Message |=  112;	
			Send_UDP(&Message);
		break;
		
                case (Turn_Mechanism_Motor_On + Linear_Actuator_Up):
			Message |= 112;
			//Send the Linear Actuator Command:
			Send_UDP(&Message);
			//Wait for a 100 milliseconds before transmitting the next command.
			usleep(Sleep_Time);
		case Turn_Mechanism_Motor_On:
			Message = Mechanism_Speed;
		case Turn_Mechanism_Motor_Off:
			Message |= 96; 
			Send_UDP(&Message);
		break;	

		case (Turn_Mechanism_Motor_Off + Linear_Actuator_Up):
			Message |= 96;
			//Send the Mechanism Command:
                        Send_UDP(&Message);
                        //Wait for a 100 milliseconds before transmitting the next command.
                        usleep(Sleep_Time);
			Message = 112;
			Send_UDP(&Message);
		break;
		
		case E_STOP:
			Message = 32;
			Send_UDP(&Message);
			return;		
		break;	
	
		case Linear_Actuator_Home:
			Message = 116;
			Send_UDP(&Message); 
		break;

		case Mechanism_Motor_Full_Reverse:
			Message = 103;
			Send_UDP(&Message);
			return;
		break;

		case Both_Wheels_Half_Forward:
			Message = 66;
			Send_UDP(&Message);
			//Wait for 100 milliseconds before transmitting the next command.
			Message = 82;
			Send_UDP(&Message); 
			return;
		break;

	}//End switch
	
//------------------------------------------------------ 
//Calculate and send the left stick values.
//------------------------------------------------------	

	Message = 64;		//Set to eGO.
	float Absolute_Deviation = fabsf(Controller_Joystick.linear.x);

	if (Controller_Joystick.linear.x < 0)							{Message |= 8;}

	if (Absolute_Deviation > 0.500)
	{
		if (Absolute_Deviation > 0.750)
		{
			if (Absolute_Deviation > 0.875) 					{Message |= 7;}
			else									{Message |= 6;}
		}//End if
		else 
		{	
			if (Absolute_Deviation > 0.625) 					{Message |= 5;}
			else									{Message |= 4;}
		}//End if 
	}//End if > 0.5
	else
	{
		if (Absolute_Deviation > 0.250)
		{
			if (Absolute_Deviation > 0.375)						{Message |= 3;}
			else									{Message |= 2;}	
		}//End if
		else		{if (Absolute_Deviation > 0.125)				{Message |= 1;}}
	}//End if <= 0.5

/**************************************************************************************************************/
/*
	if (Absolute_Deviation > 0.875)								{Message |= 7;}
	else 
	{
		if (Absolute_Deviation > 0.750)							{Message |= 6;}
		else
		{
			if (Absolute_Deviation > 0.625)						{Message |= 5;}
			else
			{
				if (Absolute_Deviation > 0.500)					{Message |= 4;}
				else
				{
					if (Absolute_Deviation > 0.375)				{Message |= 3;}
					else
					{
						if (Absolute_Deviation > 0.250)			{Message |= 2;}
						else
						{
							if (Absolute_Deviation > 0.125)		{Message |= 1;}
						}//End Power Level 1
					}//End Power Level 2
				}//End Power Level 3
			}//End Power Level 4
		}//End Power Level 5
	}//End Power Level 6
*/
/**************************************************************************************************************/

	Send_UDP(&Message);
	usleep(Sleep_Time);	//Sleep for a moment before sending the next value.


//------------------------------------------------------ 
// Calculate and send the right stick values.
//------------------------------------------------------ 

	Message = 80;           //Set to eGO.
        Absolute_Deviation = fabsf(Controller_Joystick.angular.z);

        if (Controller_Joystick.angular.z < 0)                                                  {Message |= 8;}

        if (Absolute_Deviation > 0.500)
        {
                if (Absolute_Deviation > 0.750)
                {
                        if (Absolute_Deviation > 0.875)                                         {Message |= 7;}
                        else                                                                    {Message |= 6;}
                }//End if
                else
                {
                        if (Absolute_Deviation > 0.625)                                         {Message |= 5;}
                        else                                                                    {Message |= 4;}
                }//End if 
        }//End if > 0.5
        else
        {
                if (Absolute_Deviation > 0.250)
                {
                        if (Absolute_Deviation > 0.375)                                         {Message |= 3;}
                        else                                                                    {Message |= 2;}
                }//End if
                else            {if (Absolute_Deviation > 0.125)                                {Message |= 1;}}
        }//End if <= 0.5

/**************************************************************************************************************/
/*
        if (Absolute_Deviation > 0.875)                                                         {Message |= 7;}
        else
        {
                if (Absolute_Deviation > 0.750)                                                 {Message |= 6;}
                else
                {
                        if (Absolute_Deviation > 0.625)                                         {Message |= 5;}
                        else
                        {
                                if (Absolute_Deviation > 0.500)                                 {Message |= 4;}
                                else
                                {
                                        if (Absolute_Deviation > 0.375)                         {Message |= 3;}
                                        else
                                        {
                                                if (Absolute_Deviation > 0.250)                 {Message |= 2;}
                                                else
                                                {
                                                        if (Absolute_Deviation > 0.125)         {Message |= 1;}
                                                }//End Power Level 1
                                        }//End Power Level 2
                                }//End Power Level 3
                        }//End Power Level 4
                }//End Power Level 5
        }//End Power Level 6
*/
/**************************************************************************************************************/

        Send_UDP(&Message);
        usleep(Sleep_Time);     //Sleep for a moment before sending the next value.

}//End Joystick_Callback

/*Send_UDP*/

uint8_t Send_UDP (const unsigned char * Message)
{
	ROS_INFO_STREAM("MESSAGE: " << (int)*Message << endl);
	//Variable Instantiation/Initialization:
        const char * CC3000_Address = "192.168.2.19";
        int Socket = socket(AF_INET, SOCK_DGRAM, 0);
        short int Transmission_Status = 0;
        struct sockaddr_in Address;
	struct sockaddr Address_2;
        struct hostent * Server;

        if (Socket == -1)       {ERROR("\nUnable to open socket!\n");}
        memset((char *) &Address, 0, sizeof(Address));
        Address.sin_family = AF_INET;
        Address.sin_port = htons(PORT);
        if (inet_aton(CC3000_Address, &Address.sin_addr) == 0)  {ERROR("\ninet action failed!\n");}

        //Begin UDP Test:
        ROS_INFO_STREAM("Beginning UDP TEST." << endl);

        //Send the data:
        ROS_INFO_STREAM("...Sending \'UDP_TEST_SUCCESS!\'...." << endl);

	memcpy(&Address_2, &Address, sizeof(Address_2));

      //if (sendto(Socket, Message, 1, 0, &Address, sizeof(Address)) == -1)  {ERROR("\nUnable to transmit!\n");}
        if (sendto(Socket, Message, 1, 0, &Address_2, sizeof(Address)) == -1)  {ERROR("\nUnable to transmit!\n");}
	ROS_INFO_STREAM("Message sent!" << endl);

        close(Socket);

	ROS_INFO_STREAM("Socket closed." << endl);

	usleep(Sleep_Time);
	return 0;		//Success!
}//End Send_UDP

/*ERROR*/
void ERROR (const char *msg)
{
        ROS_ERROR(msg);
        //exit(1);
}//End ERROR

