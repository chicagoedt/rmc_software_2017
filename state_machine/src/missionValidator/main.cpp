#include "mission_validator.h"
#include <signal.h>

void sigIntHandler(int sig)
{
	ROS_DEBUG("--> SIGINT Handler called <--");

	// Tell the ros node that we want to shutdown, so we receive a
	// clean exit
	ros::shutdown();
}

int main(int argc, char **argv)
{
    	ros::init(argc, argv, "mission_validator");

    	signal(SIGINT, sigIntHandler);

	MissionValidator validator;

	validator.Initialize();
	validator.Run();

	return 0;
}
