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
#include <TimerOne.h>


ros::NodeHandle  nh;
Servo myservo;  // create servo object to control a servo 
int pos;    // variable to store the servo position 
int turn = 1;  // direction to turn. 1 is clockwise, -1 is counter-clockwise, 0 is neither (don't turn).
unsigned long previous_millis = 0;
const float ERROR_THRESHOLD = 5.0f;  // in degrees
const int DELAY_INCREMENT = 50;  // in milliseconds
const int SERVO_INCREMENT = 1;
const int LED_PIN = 13;
const long LOST_TIMER_LIMIT = 2500;
long lost_count = 0; 

float error_angle = 0;  // in degrees
std_msgs::Float32 angle_message;
ros::Publisher servo_pub("error_angle", &angle_message);

bool aruco_lost;


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
  
  // reset lost timer (we are not lost anymore)

  aruco_lost = false;
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

// called whenever we haven't seen aruco in a while
void lost_callback(void)
{
  if (lost_count > LOST_TIMER_LIMIT) {
    aruco_lost = true;
//    digitalWrite(LED_PIN, HIGH-digitalRead(LED_PIN));   // blink the led
    lost_count = 0;
  } else {
    lost_count++;
//    turn = 0;
  }
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  myservo.attach(9);        // attaches the servo on pin 9 to the servo object 
  nh.initNode();            // initializes ROS node
  nh.advertise(servo_pub);
  nh.subscribe(transform_sub);
  broadcaster.init(nh);     // initializes tf broadcaster
    pos = 90;                // reference configuration
    t.header.frame_id = parent;
    t.child_frame_id = child;
    
  // set aruco_lost to true (start off being lost)
  aruco_lost = true;

  // initialize lost timer and callback
  Timer1.initialize(LOST_TIMER_LIMIT);
  Timer1.attachInterrupt(lost_callback); // blinkLED to run every 0.15 seconds
  
//  FlexiTimer2::set(500, 1.0/1000, lost_callback);
}

void loop()
{


  // no need to disable interrupts when reading aruco_lost since it's a boolean
  // sweep infinitely until we find aruco
  while (aruco_lost) {
  
  // uses a no-delay timer (cannot use delay() because TIMER1 is already in use)
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
      digitalWrite(LED_PIN, HIGH-digitalRead(LED_PIN));   // blink the led
      
      if (pos <= 0) turn = -1;
      else if (pos >= 180) turn = 1;
  
    }
  }
  // we finally found the aruco!!!

  
    // uses a no-delay timer (cannot use delay() because TIMER1 is already in use)
  unsigned long current_millis = millis();
 
  if(current_millis - previous_millis > DELAY_INCREMENT) {
    previous_millis = current_millis;   
      
    digitalWrite(LED_PIN, HIGH);   // turn LED on to indicate found aruco
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
      
}
