#ifndef State_Machine_h
#define State_Machine_h

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <rmc_simulation/PanServoAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

#include <math.h>

#include <vector>
#include <queue>
#include <sstream>
#include <stack>
#include <deque>

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
       		ros::Subscriber _arucoSub; 

		ros::Subscriber _imuSub;
		ros::Publisher  _imuPub;
		ros::Publisher  _arucoPub;
		ros::Publisher  _actuatorPub;

		float _previous_x_accel ;
		bool  _didDock;
		// Tweak below three parameters inside the launch file for optimal dock performance
		int   _values_for_average;  // Number of values considered in the 'average' calculation
		double _threshold; // The minimum difference in acceleration to be considered a hit
		int   _num_to_dock; // How many consecutive hits needed to consider dock
		int   _above_threshold_count;
		int   _numCheck; // How many more datum to check for hits
		std::deque<double> _average_imu_g; // Double ended queue that stores the imu values
		tf::StampedTransform  _tf_base_link_to_map;
		tf::TransformListener _tf_listener;


		MoveBaseClient 	_moveBaseAC;
		PanServoClient	_panServoAC;


        	void moveActuators(bool goUp);
		
		bool startConnectionAC;
		bool sendGoalToAC(geometry_msgs::Pose goalPose);
        	void servoCameraState(const std_msgs::Bool::ConstPtr& servoState); 
        	void arucoPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg);
        	void moveToGoalPoint();
		
		void dock();
		void dockCallback(const sensor_msgs::Imu::ConstPtr& msg);
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

        bool _foundMarker;


};


#endif
