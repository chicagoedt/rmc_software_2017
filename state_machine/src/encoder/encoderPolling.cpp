#include "encoderPolling.h"
#include <fstream>
#include <time.h>

void EncoderPoller::run()
{
	
	ROS_INFO_STREAM("starting");
	_encoderPub = _nh.advertise<std_msgs::Float64>("encoder_data", 1000);

	ros::Rate loop_rate(10);

	std::string portPath;
	if (_nh.hasParam("port")) {
		_nh.getParam("port", portPath);
	} else {
		portPath = "/dev/ttyACM1";
	}
	
	ROS_INFO_STREAM("connected");

	char c;
	std::fstream f;

	f.open(portPath.c_str());

	std_msgs::Bool data;

	time_t current, old;

	time(&current);
	time(&old);

	ROS_INFO_STREAM("Starting 2");

	while(ros::ok()) {
		
		ROS_INFO_STREAM("Running");
	
		int x = std::cin.peek();
		ROS_INFO_STREAM(x);
		if (x == EOF) {
			time(&current);
			double diff = difftime(old, current);
			if (diff > 1.9) data.data = false;
			else data.data = true;
		} else {
			f >> c;
			time(&old);
			data.data = true;
		}

		ROS_INFO_STREAM("Spinning?: " << data);
		_encoderPub.publish(data);
	}
}

