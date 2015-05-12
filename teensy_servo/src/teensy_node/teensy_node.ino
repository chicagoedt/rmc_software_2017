/*
 * rosserial node that sweeps the servo and publishes its position as int
 */
//#define USE_USBCON
#include <ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <ros/time.h>
#include <Servo.h>


ros::NodeHandle  nh;
Servo myservo;  // create servo object to control a servo 
int pos;    // variable to store the servo position 
int turn = 1;  // direction to turn. 1 is clockwise, -1 is counter-clockwise, 0 is neither (don't turn).
unsigned long previous_millis = 0;
const float ERROR_THRESHOLD = 5.0f;  // in degrees
const int DELAY_INCREMENT = 50;  // in milliseconds
const int SERVO_INCREMENT = 1;
const int LED_PIN = 13;
const unsigned long SWEEP_TIMER_LIMIT = 1000000;     // in microseconds
const unsigned long LOST_TIMER_LIMIT = 3000000;   // in microseconds
unsigned long time_seen_marker = millis();
bool sweeping = true;

float error_angle = 0;  // in degrees
std_msgs::Float32 angle_message;
std_msgs::Bool aruco_lost;
ros::Publisher servo_pub("error_angle", &angle_message);
ros::Publisher lost_pub("servo_camera_state", &aruco_lost);

IntervalTimer lost_timer;
IntervalTimer sweep_timer;

void transform_callback( const geometry_msgs::TransformStamped& t);
ros::Subscriber<geometry_msgs::TransformStamped> transform_sub("/ar_single_board/transform", transform_callback );


geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
geometry_msgs::Quaternion q;


const char parent[] = "/servo_mount";
const char child[] = "/camera";


void transform_callback( const geometry_msgs::TransformStamped& t){
  error_angle = atan(t.transform.translation.x / t.transform.translation.z) * 180 / 3.14159;  // in degrees
  
  
  if (error_angle > ERROR_THRESHOLD)
    turn = 1;
  else if (error_angle < - ERROR_THRESHOLD)
    turn = -1;
  else
    turn = 0;

  // if we thought we were lost (just found a marker after we thought we were lost),
  if (aruco_lost.data) {
    // publish that we're not lost
    aruco_lost.data = false;
    lost_pub.publish(&aruco_lost);
  }

  // reset lost timer (we are not lost anymore)
  sweeping = false;
  sweep_timer.begin(sweep_callback, SWEEP_TIMER_LIMIT);
  aruco_lost.data = false;
  lost_timer.begin(lost_callback, LOST_TIMER_LIMIT);
}

void broadcast_tf()
{    
    double angle = map(pos, 0, 180, -90, 90);

    // update quaternion q based on position of servo
    q = tf::createQuaternionFromYaw(angle * 3.14159 / 180);
    
    // broadcast tf
    t.transform.rotation = q;
    t.header.stamp = nh.now();
    broadcaster.sendTransform(t);
    
    // FOR DEBUGGING ONLY
    angle_message.data = error_angle;
    servo_pub.publish(&angle_message);
    nh.spinOnce();
}

void lost_callback(void)
{
  // if aruco_lost is false, set it to true
  if (!aruco_lost.data && sweeping) {
    aruco_lost.data = true;
    // publish that we're lost
    lost_pub.publish(&aruco_lost);
  }
}

void sweep_callback(void)
{
  // if sweeping is false, set it to true
  if (!sweeping) {
    sweeping = true;
  }
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  myservo.attach(9);        // attaches the servo on pin 9 to the servo object 
  nh.initNode();            // initializes ROS node
  nh.advertise(servo_pub);
  nh.advertise(lost_pub);
  nh.subscribe(transform_sub);
  broadcaster.init(nh);     // initializes tf broadcaster
  pos = 90;                // reference configuration
  t.header.frame_id = parent;
  t.child_frame_id = child;
    
  // set aruco_lost to true (start off being lost)
  aruco_lost.data = true;

  // initialize lost timer and callback
  lost_timer.begin(lost_callback, LOST_TIMER_LIMIT);
  sweep_timer.begin(sweep_callback, SWEEP_TIMER_LIMIT);

}

void loop()
{

  // sweep infinitely until we find aruco
  while (sweeping) {
  
  // uses a no-delay timer
  unsigned long current_millis = millis();
 
    if(current_millis - previous_millis > DELAY_INCREMENT) {
      previous_millis = current_millis;   
      
      if (turn == 0) turn = 1;
      
      if (turn < 0) {
        pos += SERVO_INCREMENT;
      } else {
        pos -= SERVO_INCREMENT;
      }
  
      myservo.write(pos);              // move servo to position
      broadcast_tf();
      if (pos <= 0) turn = -1;
      else if (pos >= 180) turn = 1;
  
    }
    digitalWrite(LED_PIN, aruco_lost.data);
  }
  // we finally found the aruco!!!

  
    // uses a no-delay timer (cannot use delay() because TIMER1 is already in use)
  unsigned long current_millis = millis();
 
  if(current_millis - previous_millis > DELAY_INCREMENT) {
    previous_millis = current_millis;   
      
    // turn towards the aruco board
    // counter-clockwise
    if (turn < 0 && pos < 180) {
      pos += SERVO_INCREMENT;
    // clockwise
    } else if (turn > 0 && pos > 0) {
      pos -= SERVO_INCREMENT;
    }
  
    myservo.write(pos);
    broadcast_tf();
  //  delay(DELAY_INCREMENT);
    
  }
  
  digitalWrite(LED_PIN, aruco_lost.data);

}
