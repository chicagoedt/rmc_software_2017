/*
 * rosserial node that sweeps the servo and publishes its position as int
 */
//#define USE_USBCON
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <Servo.h> 

ros::NodeHandle  nh;
Servo myservo;  // create servo object to control a servo 
int pos = 77;    // variable to store the servo position 


//void messageCb( const std_msgs::Empty& toggle_msg){
//  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
//}

//ros::Subscriber<std_msgs::Empty> sub("toggle_led", messageCb );



std_msgs::Int32 pos_msg;
ros::Publisher servo_pub("servo", &pos_msg);

void setup()
{
//  pinMode(13, OUTPUT);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  nh.initNode();
  nh.advertise(servo_pub);
//  nh.subscribe(sub);
}

void publish_spin_pause()
{
    pos_msg.data = pos;
    servo_pub.publish( &pos_msg );
    nh.spinOnce();
    delay(100);
}

void loop()
{

  for(pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    publish_spin_pause();
  } 
  for(pos = 180; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    publish_spin_pause();  
  } 

}
