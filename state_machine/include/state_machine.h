#ifndef State_Machine_h
#define State_Machine_h

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <rmc_simulation/PanServoAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>

#include <vector>
#include <queue>
#include <sstream>
#include <stack>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<rmc_simulation::PanServoAction> PanServoClient;


class StateMachineBase
{

	const int MAX_GOAL_POINTS; // Refer to Goal Points vector

	public:

		StateMachineBase(void);
		~StateMachineBase(void);

		void run();
		bool Initialize();

	private:

		ros::NodeHandle	_nh;
		ros::NodeHandle	_nhLocal;

        	ros::Subscriber _servoSub; 

		MoveBaseClient 	_moveBaseAC;
		PanServoClient	_panServoAC;


		bool startConnectionAC;
		bool sendGoalToAC(geometry_msgs::Pose goalPose);
        	void servoCameraState(const std_msgs::Bool::ConstPtr& servoState); 
        	void moveToGoalPoint();

		/*
		 * @brief 
		 *
		 * Dumping Zone: Point 1. 		This will be our 'origin' to return to dump.
		 * Digging Zone: Point 2/3/4. 	In our queue array we wont ittirate from Point 2
		 *								to Point 3 to Point 4, instead these will be our 
		 *								be each one of our dig zone end points for each 
		 *								traversial itteration (Drive, Dig, Dump). 
		 * Aligning Zone: Point 5		After we finish digging at one of our dig zone end 
		 *								points (2,3, or 4), Point 5 will be the following 
		 *								irrirated goal sent to actionlib server.
		 * Example: 	Localize --> Point 2 --> Point 5 --> Point 0 
		 *						 --> Point 3 --> Point 5 --> Point 0
		 *						 --> Point 4 --> Point 5 --> Point 0
		 */
		std::queue<geometry_msgs::Pose> _goalPointsQueue;

        enum eState
        {
            eInitializing,
            eRelocalize,
            eDriveToDig,
            eDigging,
            eDriveToDump,
            eDumping
        };

        eState _robotState;

        std::stack<eState> _stateStack;

};


#endif
