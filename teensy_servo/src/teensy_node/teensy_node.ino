/*
 * rosserial node that sweeps the servo and publishes its position as int
 */
//#define USE_USBCON
#include <ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32.h>
#include <ros/time.h>
#include <Servo.h> 

ros::NodeHandle  nh;
Servo myservo;  // create servo object to control a servo 
int pos;    // variable to store the servo position 



float error_angle = 0;  // in degrees
std_msgs::Float32 angle_message;
ros::Publisher servo_pub("error_angle", &angle_message);

void transform_callback( const geometry_msgs::TransformStamped& t){
  error_angle = atan(t.transform.translation.x / t.transform.translation.z) * 180 / 3.14159;  // in degrees
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<geometry_msgs::TransformStamped> transform_sub("/ar_single_board/transform", transform_callback );


geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
geometry_msgs::Quaternion q;


char parent[] = "/servo_mount";
char child[] = "/camera";

void setup()
{
  pinMode(13, OUTPUT);
  myservo.attach(9);        // attaches the servo on pin 9 to the servo object 
  nh.initNode();            // initializes ROS node
  nh.advertise(servo_pub);
  nh.subscribe(transform_sub);
  broadcaster.init(nh);     // initializes tf broadcaster
    pos = 0;
    t.header.frame_id = parent;
    t.child_frame_id = child;
}

void broadcast_spin_pause()
{    
    double angle = map(pos, 0, 180, -90, 90);

    // update quaternion q based on position of servo
    q = tf::createQuaternionFromYaw(angle * 3.14159 / 180);

    
    t.transform.rotation = q;
    t.header.stamp = nh.now();
    broadcaster.sendTransform(t);
    
    // FOR DEBUGGING ONLY
    angle_message.data = error_angle;
    servo_pub.publish(&angle_message);
    
    nh.spinOnce();
    delay(10);
}

void loop()
{

  for(pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    broadcast_spin_pause();
  } 
  for(pos = 180; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    broadcast_spin_pause();  
  } 

}
