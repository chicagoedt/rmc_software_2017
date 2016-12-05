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
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <roboteq_node/Actuators.h>
#include <state_machine/SensorStatus.h>
#include <state_machine/ValidateSensors.h>

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

	enum eDigPosition
	{
		eDig = 850,
		eHome = 0,
		eDump = -800
	};

	public:

		StateMachineBase(void);
		~StateMachineBase(void);

		void run();
		bool Initialize();

	private:

        	void moveActuators(bool goUp);
		
		bool startConnectionAC;
		bool sendGoalToAC(geometry_msgs::Pose goalPose);
        	void servoCameraState(const std_msgs::Bool::ConstPtr& servoState); 
        	void arucoPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg);
		bool callSensorValidator(state_machine::ValidateSensors srv);
        	bool moveToGoalPoint(geometry_msgs::Pose waypoint);
		void setActuatorPosition(eDigPosition digPosition);
		void setServoAngle(float angle);
		
		void dock();
		void undock();
		void dockCallback(const sensor_msgs::Imu::ConstPtr& msg);

	private:

		ros::NodeHandle	_nh;
		ros::NodeHandle	_nhLocal;

		ros::Subscriber _servoSub; 
		ros::Subscriber _arucoSub; 
		ros::Subscriber _imuSub;

		ros::Publisher	_servoPub;
		ros::Publisher  _imuPub;
		ros::Publisher  _digPub;
		ros::Publisher  _arucoPub;
		ros::Publisher  _actuatorPub;
		ros::Publisher  _digstatePub;

		ros::ServiceClient _actuatorClient;
		ros::ServiceClient _validatorClient;

		state_machine::SensorStatus _currentSensorRequest;

		float 	_previous_x_accel;
		bool  	_didDock;
		// Tweak below three parameters inside the launch file for optimal dock performance
		int   	_values_for_average;  		// Number of values considered in the 'average' calculation
		double 	_threshold; 			// The minimum difference in acceleration to be considered a hit
		int   	_num_to_dock; 			// How many consecutive hits needed to consider dock
		int   	_above_threshold_count;
		int   	_numCheck; 			// How many more datum to check for hits
		std::deque<double> 	_average_imu_g; // Double ended queue that stores the imu values
		tf::StampedTransform  	_tf_base_link_to_map;
		tf::TransformListener 	_tf_listener;


		MoveBaseClient 	_moveBaseAC;
		PanServoClient	_panServoAC;

		std::queue<geometry_msgs::Pose> _goalPointsQueue;

		geometry_msgs::Pose	_dockingPose;
		geometry_msgs::Pose	_digStartPose;
		geometry_msgs::Pose	_digEndPose;

		int	_currentDigCycleCount;

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

        bool _isSimulation;

};


#endif
