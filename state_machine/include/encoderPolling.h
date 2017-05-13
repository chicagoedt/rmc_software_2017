#ifndef ENCODER_POLLING_H
#define ENCODER_POLLING_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>

#include <math.h>

#include <vector>
#include <queue>
#include <sstream>
#include <stack>
#include <deque>

class EncoderPoller
{

	public:

		void run();

	private:

		ros::NodeHandle	_nh;

		ros::Publisher	_encoderPub;

};


#endif
