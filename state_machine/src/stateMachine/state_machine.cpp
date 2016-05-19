#include "state_machine.h"

#define GRAVITY 9.81

StateMachineBase::StateMachineBase(void):
	_driveSpeed(0.0), _moveBaseAC("move_base", true), _panServoAC("pan_servo", true), _nhLocal("~"), _robotState(eInitializing), _foundMarker(false)
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
/*
        if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
        {
                ros::console::notifyLoggerLevelsChanged();
        }
*/
	if (_nhLocal.hasParam("DockingPosition"))
	{
		std::vector<double>   pose;
		_nhLocal.getParam("DockingPosition", pose);
		_dockingPose.position.x = pose[0];
		_dockingPose.position.y = pose[1];
	}
	else
	{
		ROS_ERROR_STREAM("Docking Position NOT found!");
      		return false;
	}

	if (_nhLocal.hasParam("DigStart"))
	{
		std::vector<double>   pose;
		_nhLocal.getParam("DigStart", pose);
		_digStartPose.position.x = pose[0];
		_digStartPose.position.y = pose[1];
	}
	else
	{
		ROS_ERROR_STREAM("Dig Start Position NOT found!");
      		return false;
	}

	if (_nhLocal.hasParam("DigDistance"))
	{
		double digDistance = 0;
		_nhLocal.getParam("DigDistance", digDistance);
		_digEndPose.position.x = _digStartPose.position.x + digDistance;
	}
	else
	{
		ROS_ERROR_STREAM("Dig Distance NOT found!");
      		return false;
	}

	
	_digDriveSpeed = 0.25;
	_nhLocal.param<double>("dig_drive_speed", _digDriveSpeed);

	_isSimulation = false;
	_nhLocal.param<bool>("simulation", _isSimulation);

	if(!_isSimulation)
	{
		_currentSensorRequest.roboteq = true;
		_nhLocal.param<bool>("Roboteq", _currentSensorRequest.roboteq);

		_currentSensorRequest.servo = true;
		_nhLocal.param<bool>("Servo", _currentSensorRequest.servo);
	}
	else
	{
		_currentSensorRequest.roboteq = false;
		_currentSensorRequest.servo = false;
	}

	_currentSensorRequest.rtab = true;
	_nhLocal.param<bool>("RTAB", _currentSensorRequest.rtab);

	_currentSensorRequest.aruco = true;
	_nhLocal.param<bool>("Aruco", _currentSensorRequest.servo);

	_turnStartLeft = false;
	_turnStartRight = false;

	_nhLocal.param<bool>("turn_start_left", _turnStartLeft);
	_nhLocal.param<bool>("turn_start_right", _turnStartRight);

	_currentDigCycleCount = 0;

	_servoPub = _nh.advertise<std_msgs::Float64>("blackfly_mount_joint/command", 5);
	_digPub = _nh.advertise<std_msgs::Float64>("dig_vel", 1);
  	_arucoSub = _nh.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("ar_single_board/pose", 1, &StateMachineBase::arucoPoseCallback, this);
  	_actuatorClient = _nh.serviceClient<roboteq_node::Actuators>("set_actuators"); // need to figure out how to not make roboteq_node a dependency to state_machine which can also be used by the simulator
	_validatorClient = _nh.serviceClient<state_machine::ValidateSensors>("validate_sensors");
	_poseFollowerClient = _nh.serviceClient<pose_follower::SetMaxVelocity>("set_max_velocity");
	_rtabClient = _nh.serviceClient<std_srvs::Empty>("rtabmap/reset_odom");

	_arucoPub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
/*
	if(initializeServo())
		ROS_INFO("Servo Initialized.");
	else
		ROS_ERROR("TF of Servo not changing!");
*/
  	return true;
}

void StateMachineBase::arucoPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg)
{
	//ROS_INFO_STREAM("Aruco Pose X: " << poseMsg->pose.pose.position.x << " Y: " << poseMsg->pose.pose.position.y);
  	_foundMarker = true;
  //_panServoAC.stopTrackingGoal();
}

void StateMachineBase::setActuatorPosition(eDigPosition digPosition)
{
	if(!_isSimulation)
	{
		roboteq_node::Actuators srv;
		srv.request.actuator_position = digPosition;

		if(_actuatorClient.call(srv))  // blocking call
		{
			ROS_INFO_STREAM("Moved/Moving to " << digPosition);
		}
		else
		{
			ROS_ERROR_STREAM("Failed to call actuator service! Check if roboteq's are on...");
		}
	}
	else
	{
		ROS_INFO("Simulation Actuators to position.");
	}

}

void StateMachineBase::setServoAngle(float angle)
{
	// angle is in radians
	std_msgs::Float64 servoAngle;
	servoAngle.data = angle;

	_servoPub.publish(servoAngle);	
	ros::spinOnce();
}

void StateMachineBase::updateCurrentSpeed(float driveSpeed)
{
	pose_follower::SetMaxVelocity srv;
	srv.request.max_linear_vel = driveSpeed;	
	
	ROS_INFO("Updating speed...");	

	if(_poseFollowerClient.call(srv))
	{
		ROS_INFO_STREAM("Setting drive speed to: " << driveSpeed);
	}
	else
	{
		ROS_ERROR("Cannot set new drive speed!");	
	}
}

bool StateMachineBase::moveToGoalPoint(geometry_msgs::Pose waypoint)
{
  	while(!_moveBaseAC.waitForServer(ros::Duration(3.0)))
	{
    		ROS_INFO("Waiting for the move_base action server to come up");
  	}

	move_base_msgs::MoveBaseGoal moveBaseGoal;
        
        moveBaseGoal.target_pose.header.frame_id   = "map";
        moveBaseGoal.target_pose.header.stamp      = ros::Time::now();

        moveBaseGoal.target_pose.pose.position     = waypoint.position;
        moveBaseGoal.target_pose.pose.orientation  = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

        ROS_INFO_STREAM("Sending goal(X, Y):" << "[ " << moveBaseGoal.target_pose.pose.position.x << " , " << moveBaseGoal.target_pose.pose.position.y << " ]");

        _moveBaseAC.sendGoal(moveBaseGoal);

        ROS_INFO("Sent Goal...");

        //_moveBaseAC.waitForResult();

        //ROS_INFO("Got result...");


	while(_moveBaseAC.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		if(_robotState == eDigging)
		{
			std_msgs::Float64 digVel;
			digVel.data = 450;
			_digPub.publish(digVel);
			ROS_WARN_THROTTLE(1, "Digging...");
		}	
		else
		{
			ROS_INFO_THROTTLE(1, "Moving to waypoint...");
		}
		ros::spinOnce();
		ros::Duration(0.1).sleep();	
	}
/*
        if(_moveBaseAC.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
                ROS_INFO("Succesfully moved to GoalPoint.");
		return true;
        }
	else
	{
		ROS_ERROR("Failed to move to GoalPoint.");
		return false;
	}
*/
}

bool StateMachineBase::callSensorValidator(state_machine::ValidateSensors srv)
{
	if(_validatorClient.call(srv))
	{

		ROS_WARN("--------------------------------------------------");
		// bunch of if's so that status print statements are color coordinated
		// TODO: Its printing out all of them false, even though at least one should be true
		if(srv.request.sensors.roboteq)
			if(srv.response.status.roboteq == true)
				ROS_INFO("Roboteq = True");
			else
				ROS_ERROR("Roboteq = False");
		else
			ROS_WARN("Roboteq = Unknown");

		if(srv.request.sensors.aruco)
			if(srv.response.status.aruco == true)
				ROS_INFO("Aruco TF = True");
			else
				ROS_WARN("Aruco TF = False");
		else
			ROS_WARN("Aruco = Unknown");

		if(srv.request.sensors.servo)
			if(srv.response.status.servo == true)
				ROS_INFO("Servo = True");
			else
				ROS_ERROR("Servo = False");
		else
			ROS_WARN("Servo = Unknown");

		if(srv.request.sensors.rtab)
			if(srv.response.status.rtab == true)
				ROS_INFO("RTAB = True");
			else
				ROS_ERROR("RTAB = False");
		else
			ROS_WARN("RTAB = Unknown");
		
		ROS_WARN("--------------------------------------------------");

		if(srv.response.validated)
		{
			ROS_INFO("-- Sensors Initialized and Validated --"); return true;
		}
		else
		{
			ROS_WARN("-- Sensors could not be initialized. Exiting... --");
		}
	}

}

void StateMachineBase::findAruco(const char c){
    ros::Rate loop_rate(10);
    tf::StampedTransform board2base;
    unsigned long time = 0;
    ROS_INFO_STREAM("To upper c: " << toupper(c)); 
    switch (toupper(c)){
        case 'E':
	    {
            bool foundTransform = false;
            while (!foundTransform){
                try {
                    _tf_listener.lookupTransform("/ar_board_marker", "/base_link", ros::Time(0), board2base);
                    foundTransform = true;
                    std_srvs::Empty srv;
                    _rtabClient.call(srv);

			// rosservice call /rtabmap/reset_odom
                }
                catch (tf::TransformException ex) {
                    geometry_msgs::Twist msg;
                    msg.angular.z = ((time % 70) < 45) ? -0.9 : 0.0;
                    _arucoPub.publish(msg);
                    ros::spinOnce();
                    loop_rate.sleep();
		    time++;
                }
            }
            geometry_msgs::Twist msg;
            msg.angular.z = 0;
            _arucoPub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
            break;
	    }
        case 'N':
	    {
            bool foundTransform = false;
            setServoAngle(1.57);
            while (!foundTransform){
                try {
                    _tf_listener.lookupTransform("/ar_board_marker", "/base_link", ros::Time(0), board2base);
                    foundTransform = true;
                    std_srvs::Empty srv;
                    _rtabClient.call(srv);
                }
                catch (tf::TransformException ex) {
                    geometry_msgs::Twist msg;
                    msg.angular.z = ((time % 70) < 45) ? 0.9 : 0.0;
                    _arucoPub.publish(msg);
                    ros::spinOnce();
                    loop_rate.sleep();
		    time++;
                }
            }
            geometry_msgs::Twist msg;
            msg.angular.z = 0;
            _arucoPub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
	    setServoAngle(0);
            break;
		}
	default:
		ROS_WARN("Not facing any direction!");
		break;
	}
}
	


void StateMachineBase::run()
{
  	//ROS_INFO("Sleeping for 2 seconds...");
 	//ros::Duration(3.0).sleep();
 	ROS_INFO("Starting!");
/*
	// TODO: add request component to ValidateSensors message so that we can request specific
	//       validations (all, or individual)
	state_machine::ValidateSensors srv;
	if (!_isSimulation) 
	{
		srv.request.sensors.roboteq 	= _currentSensorRequest.roboteq;
		srv.request.sensors.rtab 	= _currentSensorRequest.rtab;
		srv.request.sensors.servo 	= _currentSensorRequest.servo; 
		srv.request.sensors.aruco 	= _currentSensorRequest.aruco;

		callSensorValidator(srv);
	}

	std::string orient;
        _nh.getParam("starting_orientation", orient);

        if (srv.response.status.aruco == false)
	{
                if (orient == "north") 
			findAruco('N');
                else if (orient == "east") 
			findAruco('E');
                else if (orient == "west") 
			findAruco('W');
                else if (orient == "south") 
			findAruco('S');
		else
			ROS_WARN("No direction selected!");
        }

	ROS_INFO_STREAM("Got here.. facing: " << orient);

	while(1)
	{
		setActuatorPosition(eHome);


		ros::Duration(2.0).sleep();

		_robotState = eDriveToDig; // Start digging motors

		if(moveToGoalPoint(_digStartPose))
		{
			updateCurrentSpeed(_digDriveSpeed); // should be 0.25
			setActuatorPosition(eDig);

			ROS_INFO("Lowering Actuators to Dig...");

			if(!_isSimulation)
				ros::Duration(18.0).sleep();
			else
				ros::Duration(1.0).sleep();

			_robotState = eDigging; // Start digging motors

			if(moveToGoalPoint(_digEndPose))
			{
				updateCurrentSpeed(0); // go back to default drive speed (zero is equivalent)
				setActuatorPosition(eHome);
				ROS_INFO("Raising Actuators to Home Position...");

				if(!_isSimulation)
					ros::Duration(5.0).sleep();
				else
					ros::Duration(1.5).sleep();

				_robotState = eDriveToDig;

				if(moveToGoalPoint(_dockingPose))
				{
					_robotState = eDumping;
					setActuatorPosition(eDump);
					dock();

					ROS_INFO("Raising Actuators to Dump Position...");

					if(!_isSimulation)
						ros::Duration(20.0).sleep();
					else
						ros::Duration(2.0).sleep();

					undock();
				}
			}
		}			
  	}
*/
	dock();
	ros::Duration(11.0).sleep();
	undock();
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

void StateMachineBase::undock()
{
    //int xvel = 0;
    ros::Publisher undockPub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    tf::TransformListener tfListener(ros::Duration(1.0));
    tf::StampedTransform map2base;
    ros::Rate loop_rate(23);

    ROS_INFO("Un-docking...");

    bool foundTransform = 0;
    geometry_msgs::Twist msg;
    ros::spinOnce();
    loop_rate.sleep();
    while (!foundTransform)
    {
        try
        {
	    tfListener.waitForTransform("/odom", "/base_link", ros::Time::now(), ros::Duration(0.1));
            tfListener.lookupTransform("/odom", "/base_link", ros::Time(0), map2base/*transform*/);
            foundTransform = 1;
            ROS_INFO("Found map->base_link transform!");
        }
        catch (tf::TransformException ex){
            ROS_INFO_STREAM("Not found");
            msg.linear.x = .25;
            undockPub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    msg.linear.x = 0;
    undockPub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
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

		//_imuSub = _nh.subscribe("imu/data", 1, &StateMachineBase::dockCallback, this);

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
					_tf_listener.lookupTransform("/base_link", "/ar_board_marker",  
										   ros::Time(0), _tf_base_link_to_map/*transform*/);
					foundTransform = 1;
				}
				catch (tf::TransformException ex){
				  ROS_ERROR("Docking Error: %s",ex.what());
				
				  msg.linear.x = 0;
				  _arucoPub.publish(msg);
				  ros::spinOnce();
				  ros::Duration(0.1).sleep();
				}	
			}

			double x = _tf_base_link_to_map.getOrigin().getX();
			double y = _tf_base_link_to_map.getOrigin().getY();
			double z = _tf_base_link_to_map.getOrigin().getZ();
			tf::Quaternion rotation = _tf_base_link_to_map.getRotation();
			
			double roll, pitch, yaw;
			tf::Matrix3x3(rotation).getRPY(roll, pitch, yaw);

			yaw += 1.57;

			double absYaw = (yaw > 0) ? yaw : -1*yaw;

			if (absYaw > .02) msg.angular.z = (yaw > 0) ? .15 : -.15;

			//ROS_INFO_STREAM("Transform: " << x << " " << y << " " << z);
			ROS_INFO_STREAM("Transform difference: " << ((x-0.1) - arucoDistance));
			//ROS_INFO_STREAM("Rotation:  " << roll << " " << pitch << " " << yaw);
			msg.linear.x = -0.24;
			
			_arucoPub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
			x = ( x > 0) ? x : -1*x ;

			if ((x) < arucoDistance){
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

