/*
 * rosserial node that sweeps the servo and publishes its position as int
 */
//#define USE_USBCON
#include <ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
# include <geometry_msgs/Quaternion.h>
#include <ros/time.h>
#include <Servo.h> 

ros::NodeHandle  nh;
Servo myservo;  // create servo object to control a servo 
int pos;    // variable to store the servo position 


//void messageCb( const std_msgs::Empty& toggle_msg){
//  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
//}

//ros::Subscriber<std_msgs::Empty> sub("toggle_led", messageCb );



geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
geometry_msgs::Quaternion q;

char parent[] = "/servo_mount";
char child[] = "/camera";

void setup()
{
//  pinMode(13, OUTPUT);
  myservo.attach(9);        // attaches the servo on pin 9 to the servo object 
  nh.initNode();            // initializes ROS node
  broadcaster.init(nh);     // initializes tf broadcaster
//  nh.subscribe(sub);
    pos = 0;
    t.header.frame_id = parent;
    t.child_frame_id = child;
}

void publish_spin_pause()
{    
    double angle = map(pos, 0, 180, -90, 90);

    // update quaternion q based on position of servo
    q = tf::createQuaternionFromYaw(angle * 3.14159 / 180);

    
    t.transform.rotation = q;
    t.header.stamp = nh.now();
    broadcaster.sendTransform(t);
    nh.spinOnce();
    delay(10);
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
