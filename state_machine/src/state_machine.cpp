#include "state_machine.h"

#define GRAVITY 9.81

StateMachineBase::StateMachineBase(void):
	_moveBaseAC("move_base", true), _panServoAC("pan_servo", true), _nhLocal("~"),
   MAX_GOAL_POINTS(12), _robotState(eInitializing), _foundMarker(false)
{
	_previous_x_accel = 0;
	_didDock = 0;
	_numCheck = 5;
	_average_imu_g = std::deque<double>();
}

StateMachineBase::~StateMachineBase(void)
{

}

bool StateMachineBase::Initialize()
{
  std::string   goalParamName = "GoalPoint";

  for(int i = 0; i < MAX_GOAL_POINTS; i++)
  {
    std::ostringstream gp; gp << "GoalPoint" << i;

    ROS_DEBUG_STREAM("Loaded param: " << gp.str());

    if (_nhLocal.hasParam(gp.str()))
    {
      std::vector<double>   goalXY;

      geometry_msgs::Pose   tmpPose;

      _nhLocal.getParam(gp.str(), goalXY);

      tmpPose.position.x  = goalXY[0];
      tmpPose.position.y  = goalXY[1];
      //tf::quaternionTFToMsg(tf::createIdentityQuaternion(), tmpPose.orientation);
      //tmpPose.orientation = tf::createQuaternionMsgYaw(3.14159);

      _goalPointsQueue.push(tmpPose);
    }
    else
    {
      ROS_ERROR_STREAM("Please fill the goalpoints.yaml for GoalPoint" << i);
      return false;
    }
  }

//  _servoSub = _nh.subscribe<std_msgs::Bool> ("servo_camera_state", 1, &StateMachineBase::servoCameraState, this);
  _arucoSub = _nh.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("ar_single_board/pose", 1, &StateMachineBase::arucoPoseCallback, this);

  return true;
}

void StateMachineBase::arucoPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg)
{
  ROS_INFO_STREAM("Pose X: " << poseMsg->pose.pose.position.x << " Y: " << poseMsg->pose.pose.position.y);
  _foundMarker = true;
  //_panServoAC.stopTrackingGoal();
}

void StateMachineBase::servoCameraState(const std_msgs::Bool::ConstPtr& servoStateOK)
{
	// if servoState = true meaning the aruco marker is found
	if( servoStateOK->data )
	{

	}
	else // marker is not found
	{
		_stateStack.push(_robotState);

		// Stack current goal, drive backward 0.5 meters, if we stil dont get it,
		// then just rely on imu
		if( _robotState	== StateMachineBase::eRelocalize)
		{

		}
	}
}

void StateMachineBase::moveToGoalPoint()
{

}

void StateMachineBase::run()
{
  ROS_INFO("Sleeping for 5 seconds...");
  ros::Duration(2.0).sleep();
  ROS_INFO("Starting!");

  /*
  while(!_panServoAC.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the pan_servo action server...");
  }
  ROS_INFO("Established Connection with pan_servo ActionServer!");



  // rmc_simulation::PanServoGoal goalRequest;
  // actionlib::SimpleClientGoalHandle<rmc_simulation::PanServoAction> cgh = _panServoAC.sendGoal(goalRequest);

  rmc_simulation::PanServoGoal goalRequest;
  _panServoAC.sendGoal(goalRequest);

  ROS_INFO("Sent PanServoGoal, waiting for result...");

  while(_panServoAC.waitForResult(ros::Duration(0.1)))
  {
    ros::Duration(0.4).sleep();
    ros::spinOnce();
    if(_foundMarker)
    {
      ROS_INFO("Found marker. Exiting loop...");
      break;
    }
  }


  ROS_INFO("Got result...");


  while(!_moveBaseAC.waitForServer(ros::Duration(2.0)))
  {
    ROS_INFO("Waiting for the move_base action server...");
  }
  ROS_INFO("Established Connection with move_base ActionServer!");
  
	while(!_goalPointsQueue.empty())
	{
    move_base_msgs::MoveBaseGoal moveBaseGoal;
	
		moveBaseGoal.target_pose.header.frame_id   = "map";
		moveBaseGoal.target_pose.header.stamp      = ros::Time::now();

		moveBaseGoal.target_pose.pose.position     = _goalPointsQueue.front().position;
		moveBaseGoal.target_pose.pose.orientation  = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

		ROS_INFO_STREAM("Sending goal(X, Y):" << "[ " << moveBaseGoal.target_pose.pose.position.x << " , "
                                              		<< moveBaseGoal.target_pose.pose.position.y << " ]");

		_moveBaseAC.sendGoal(moveBaseGoal);

    ROS_INFO("Sent Goal...");

		_moveBaseAC.waitForResult();

    ROS_INFO("Got result...");

		if(_moveBaseAC.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
	      ROS_INFO("Removing goal from queue...");
  			_goalPointsQueue.pop();
  			ROS_INFO("Succesfully moved to GoalPoint.");
		}
		else
		{
  			ROS_WARN("Failed to move to GoalPoint.");
		}
	}*/

	StateMachineBase::dock();
  
}

void StateMachineBase::moveActuators(bool goUp)
{
    ROS_INFO("ACTUATORS MOVING");
    _actuatorPub = _nh.advertise<std_msgs::Float64>("joint1_position_controller/command", 1);
    std_msgs::Float64 msg;
    msg.data = goUp ? 1.9 : 0.0;
    ros::Rate loop_rate(10);

    for (int i = 0; i < 10; i++){
        _actuatorPub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void StateMachineBase::dockCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	double roll, pitch, yaw;
	float x = msg->orientation.x;
	float y = msg->orientation.y;
	float z = msg->orientation.z;
	float w = msg->orientation.w;

	tf::Quaternion q(x,y,z,w); tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	//ROS_INFO_STREAM("RPY: " << roll << " " << pitch << " " << yaw);
	double xImuAcceleration = msg->linear_acceleration.x;
	double xRealAcceleration = xImuAcceleration-GRAVITY*sin(pitch);
	//ROS_INFO_STREAM("Pitch: "<<pitch);
	//ROS_INFO_STREAM("Size: " << _average_imu_g.size() << " Accel: " << xRealAcceleration<<"    "<<_above_threshold_count);
	if (_average_imu_g.size() < _values_for_average)
	{
		//ROS_INFO_STREAM("IM HERE");
		_average_imu_g.push_back(xRealAcceleration);
	}
	else
	{
		double average = 0; double total = 0; double count = _average_imu_g.size();
		int i;
		for (i = 0; i < _average_imu_g.size(); i++)
		{
			//ROS_INFO_STREAM("g: "<<_average_imu_g[i]);
			total+=_average_imu_g[i];
		}

		average = total/count;
		//ROS_INFO_STREAM("Average x accel: " << average);
		if (xRealAcceleration > _threshold)
		{
			_above_threshold_count++;
			if (_above_threshold_count >= _num_to_dock) {_didDock = 1; ROS_INFO_STREAM("Docked with imu");}
		}
		else
		{
			//ROS_INFO_STREAM("Else reached " << xRealAcceleration);
			_average_imu_g.pop_front();
			_average_imu_g.push_back(xRealAcceleration);
			if (_above_threshold_count > 0) _numCheck--;
			if (_numCheck == 0) {_above_threshold_count = 0; _numCheck = 5;}
		}
	}
	//ROS_INFO("Callback Dock.");
	//float current_x_accel = msg->linear_acceleration.x;
	//if ((current_x_accel - _previous_x_accel) > 4.5){
	//	_didDock = 1;
	//}
	//_previous_x_accel = current_x_accel;
}

void StateMachineBase::dock()
{
	int useAruco = 0;
	double arucoDistance = 0;
	_nh.param("values_for_average", _values_for_average, 15);
	_nh.param("threshold_for_average", _threshold, 1.2);
	_nh.param("num_to_dock", _num_to_dock, 5);
	_nh.param("useAruco", useAruco, 0);
	_nh.param("arucoDistance", arucoDistance, 1.01);
	if (useAruco) ROS_INFO("USING ARUCO!");
	if (!useAruco)
	{

		ROS_INFO_STREAM("DOCKING." << _average_imu_g.size() << " ");

		_imuSub = _nh.subscribe("imu/data", 1, &StateMachineBase::dockCallback, this);
		_imuPub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

		_above_threshold_count = 0;
		ros::Rate loop_rate(250);
		ros::spinOnce();

		if (!_didDock) 
		{
			while (!_didDock)
			{
				geometry_msgs::Twist msg;
				msg.linear.x = -.1;
				_imuPub.publish(msg);
				ros::spinOnce();
				loop_rate.sleep();
			}
		}
		ROS_INFO("Finished Docking");
		geometry_msgs::Twist msg;
		msg.linear.x = 0;
		_imuPub.publish(msg);
		ros::spinOnce();
	}
	else 
	{
		ros::Rate loop_rate(250);
		ros::spinOnce();
		_arucoPub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
// testing failsafe system
		//ROS_INFO_STREAM("DOCKING." << _average_imu_g.size() << " ");

		_imuSub = _nh.subscribe("imu/data", 1, &StateMachineBase::dockCallback, this);

		_above_threshold_count = 0;
// end testing
		while (!_didDock)
		{
			geometry_msgs::Twist msg;
			msg.linear.x = -.1;
			msg.angular.z = 0;
		    tf::StampedTransform transform;
			bool foundTransform = 0;
			while (!foundTransform){
				try
				{
					_tf_listener.lookupTransform("/map", "/base_link",  
										   ros::Time(0), _tf_base_link_to_map/*transform*/);
					foundTransform = 1;
				}
				catch (tf::TransformException ex){
				  ROS_ERROR("%s",ex.what());
				  msg.linear.x = 0;
				  ros::spinOnce();
				  ros::Duration(0.5).sleep();
				}	
			}

			double x = _tf_base_link_to_map.getOrigin().getX();
			double y = _tf_base_link_to_map.getOrigin().getY();
			double z = _tf_base_link_to_map.getOrigin().getZ();
			tf::Quaternion rotation = _tf_base_link_to_map.getRotation();
			
			double roll, pitch, yaw;
			tf::Matrix3x3(rotation).getRPY(roll, pitch, yaw);

			double absYaw = (yaw > 0) ? yaw : -1*yaw;

			if (absYaw > .02) msg.angular.z = (yaw > 0) ? -.15 : .15;

			//ROS_INFO_STREAM("Transform: " << x << " " << y << " " << z);
			//ROS_INFO_STREAM("Rotation:  " << roll << " " << pitch << " " << yaw);
			msg.linear.x = -.1;
			
			_arucoPub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();

			if (x < arucoDistance){
				_didDock = 1;
				ROS_INFO_STREAM("Docked with ArUco");
				msg.linear.x = 0;
				_arucoPub.publish(msg);
				ros::spinOnce();
				loop_rate.sleep();
			}

		}
	}
}
