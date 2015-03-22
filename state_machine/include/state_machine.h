#ifndef State_Machine_h
#define State_Machine_h

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>

#include <vector>
#include <sstream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class StateMachineBase
{

	const int MAX_GOAL_POINTS; // Refer to Goal Points vector

	public:

		StateMachineBase(void);
		~StateMachineBase(void);

		void run();
		bool Initialize();

	private:

		ros::NodeHandle					_nh;

		MoveBaseClient 					_moveBaseAC;
		move_base_msgs::MoveBaseGoal 	_moveBaseGoal;

		bool startConnectionAC;
		bool sendGoalToAC(geometry_msgs::Pose goalPose);


		/*
		 * @brief 
		 *
		 * Dumping Zone: Point 1. 		This will be our 'origin' to return to dump.
		 * Digging Zone: Point 2/3/4. 	In our vector array we wont ittirate from Point 2
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
		std::vector<geometry_msgs::Pose> _goalPoints;

};


#endif