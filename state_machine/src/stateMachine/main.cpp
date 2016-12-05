#include "state_machine.h"
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
    ros::init(argc, argv, "state_machine");

    StateMachineBase stateMachine;

    signal(SIGINT, sigIntHandler);

    if (stateMachine.Initialize())
    	stateMachine.run();

	return 0;
}
