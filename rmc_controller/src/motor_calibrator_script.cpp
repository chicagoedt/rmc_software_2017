    #include "ros/ros.h"
    #include "std_msgs/String.h" 
    #include <sstream>
     
     /**
      * This demonstrates simple sending of messages over the ROS system.
       */

      void  Ask_user_right(char user);

    int main(int argc, char **argv)
     {
       /**
        * The ros::init() function needs to see argc and argv so that it can perform
        * any ROS arguments and name remapping that were provided at the command line. For 		programmatic
        * remappings you can use a different version of init() which takes remappings
        * directly, but for most command-line programs, passing argc and argv is the easiest
        * way to do it.  The third argument to init() is the name of the node.
        *
        * You must call one of the versions of ros::init() before using any other
        * part of the ROS system.
        */
       ros::init(argc, argv, "calibrator");
     
       /**
        * NodeHandle is the main access point to communications with the ROS system.
        * The first NodeHandle constructed will fully initialize this node, and the last
        * NodeHandle destructed will close down the node.
        */
       ros::NodeHandle n;
     
       /**
        * The set_wheels_power() function is how you tell ROS that you want to
        * publish on a given topic name. This invokes a call to the ROS
        * master node, which keeps a registry of who is publishing and who
        * is subscribing. After this advertise() call is made, the master
        * node will notify anyone who is trying to subscribe to this topic name,
        * and they will in turn negotiate a peer-to-peer connection with this
        * node.  set_wheels_power() returns a Publisher object which allows you to
        * publish messages on that topic through a call to publish().  Once
        * all copies of the returned Publisher object are destroyed, the topic
        * will be automatically unadvertised.
        *
        * The second parameter to set_wheels_power() is the size of the message queue
        * used for publishing messages.  If messages are published more quickly
        * than we can send them, the number here specifies how many messages to
        * buffer up before throwing some away.
        */
       ros::Publisher pub = n.set_wheels_power<base_controller::wheelpowermsg>("set_wheels_power", 1000);
     
       ros::Rate loop_rate(10);
     
       /**
        * A count of how many messages we have sent. This is used to create
        * a unique string for each message.
        */
       int count = 0;
       while (ros::ok())
       {
          
          while(Ask_user_right());

          while(Ask_user_left());

         /**
          * The publish() function is how you send messages. The parameter
          * is the message object. The type of this object must agree with the type
          * given as a template parameter to the set_wheels_power<>() call, as was done
          * in the constructor above.
         */
        // power.publish(msg);
    
        // ros::spinOnce();
  //ask for user input then waits one min using ros time
  	uint8_t Ask_user_right(char user){
  	

    std::cout<<"prepare right motor controller for calibration. Hit enter when ready"<<std::endl;
  	
    std::String userInput;
    
    std::cin >> userInput;

    base_controller::wheelpowermsg theMessage;

    theMessage.leftWheelPercentage = 0;
    theMessage.rightWheelPercentage = 1;

    pub.publish(theMessage);

    ros::Duration(1.0).sleep(); 

    theMessage.leftWheelPercentage = 0;
    theMessage.rightWheelPercentage = 0;

    pub.publish(theMessage);

    ros::Duration(1.0).sleep(); 

    theMessage.leftWheelPercentage = 0;
    theMessage.rightWheelPercentage = -1;

    pub.publish(theMessage);

    ros::Duration(1.0).sleep();

    theMessage.leftWheelPercentage = 0;
    theMessage.rightWheelPercentage = 0;

    pub.publish(theMessage);


  	std::cout<<"was the right wheel calibration successful? Y/n"
  	while (1){

  	   if(user == 'y' || 'Y')
   	     return 1; // returns true
  		break; //breaks out of loop of user enters yes
  	//continues going if user inputs No
  	   else if(user == 'n' || 'N')
  	     return 0; // returns false

  	    
    } // end of while loop
  }  //end of function


  uint8_t Ask_user_left(){
    

    ROS_INFO("prepare left motor controller for calibration. Hit enter when ready");
    
    char userInput;
    
    std::cin.get(userInput);

    if(userInput != '\n')
    {
      ROS_INFO("DUMBASS, I said hit enter!!! Restarting!");
      return 0;
    }

    base_controller::wheelpowermsg theMessage;

    theMessage.leftWheelPercentage = 1;
    theMessage.rightWheelPercentage = 0;

    pub.publish(theMessage);

    ros::Duration(1.0).sleep(); 

    theMessage.leftWheelPercentage = 0;
    theMessage.rightWheelPercentage = 0;

    pub.publish(theMessage);

    ros::Duration(1.0).sleep(); 

    theMessage.leftWheelPercentage = -1;
    theMessage.rightWheelPercentage = 0;

    pub.publish(theMessage);

    ros::Duration(1.0).sleep();

    theMessage.leftWheelPercentage = 0;
    theMessage.rightWheelPercentage = 0;

    pub.publish(theMessage);


    std::cout<<"was the right wheel calibration successful? Y/N"

    std::cin >> userInput;

    while (1){

       if(userInput[0] == 'y' || 'Y')
         return 1; // returns true
      break; //breaks out of loop of user enters yes
    //continues going if user inputs No
       else if(userInput[0] == 'n' || 'N')
         return 0; // returns false

        
    }
