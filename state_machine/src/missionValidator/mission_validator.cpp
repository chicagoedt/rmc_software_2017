#include "mission_validator.h"

MissionValidator::MissionValidator(void): _loopRate(1)
{
}

MissionValidator::~MissionValidator(void)
{
}

void MissionValidator::Initialize(void)
{
	_servoPub 	= _nh.advertise<std_msgs::Float64>("blackfly_mount_joint/command", 1);
	_arucoSub 	= _nh.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("ar_single_board/pose", 1, &MissionValidator::arucoPoseCallback, this);

	_service 	= _nh.advertiseService("validate_sensors", &MissionValidator::validationServiceCallback, this); 
	_actuatorClient = _nh.serviceClient<roboteq_node::Actuators>("set_actuators");

	_eStatus = eInvalidated; // We have not yet validated any sensors

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
	{
   		ros::console::notifyLoggerLevelsChanged();
	}

}

void MissionValidator::Run(void)
{
	while(ros::ok())
	{
		_loopRate.sleep(); // We dont want to subscribe to message as often as they are published
		ros::spinOnce();
	}	
}

bool MissionValidator::validateSensors(void)
{
	_eStatus = eValidated; // Start off validated, to reset status from previous validation

	if(!validateServo())
		_eStatus = eInvalidated;

	if(!validateHardware())
		_eStatus = eInvalidated;

	if(!validateRtab())
		_eStatus = eInvalidated;

	if(!_currentStatusMsg.aruco)
		_eStatus = eInvalidated;

	return _eStatus == eValidated;
}

bool MissionValidator::validateRtab(void)
{
	_currentStatusMsg.rtab = true; 	// assume true at start. if an exception for the tf lookup is thrown,
					// this will turn false. Otherwise it will return true	

	tf::StampedTransform    odomTf;
        try
        {
                ROS_DEBUG("--Looking up odom->base_link transform--");
                _tfListener.waitForTransform("odom","base_link", ros::Time(0), ros::Duration(3));
                _tfListener.lookupTransform("odom","base_link", ros::Time(0), odomTf);
		ROS_DEBUG("--Rtabmap Validated--");
		
        }
        catch (tf::TransformException &ex)
        {
                _currentStatusMsg.rtab = false;
                ROS_ERROR("%s",ex.what());
                ROS_ERROR("--Rtabmap In-Validated--");
        }


	return _currentStatusMsg.rtab;
}

bool MissionValidator::validateServo(void)
{
	ROS_WARN("--Validating Servo--");

	std_msgs::Float64 	servoAngle;
        tf::StampedTransform 	servoTf;


	_currentStatusMsg.servo = true; // reset and assume true, unless an error makes it false

	ROS_DEBUG("--Publishing servo position--");
	servoAngle.data = 1.0;
	_servoPub.publish(servoAngle);
	ros::spinOnce();
	ros::Duration(4).sleep();

	try
	{
		ROS_DEBUG("--Looking up servo_mount_link transform--");
		_tfListener.waitForTransform("servo_mount_link","blackfly_mount_link", ros::Time(0), ros::Duration(3));
		_tfListener.lookupTransform("servo_mount_link","blackfly_mount_link", ros::Time(0), servoTf);
		if(getYaw(servoTf.getRotation()) > 0.8) // should be 1, but 
		{
			ROS_DEBUG("--TF Lookup Successful--");
			servoAngle.data = 0.0; // double check that we really are moving the servo
			_servoPub.publish(servoAngle);
			ros::spinOnce();	// Doing this before print statement as final check 
						// that you should do, by looking at the robot and
						// see that servo is now 0
			ROS_INFO("--Servo Validated--");
			_currentStatusMsg.servo = true;	
			
		}
	}
	catch (tf::TransformException &ex)
	{
		_currentStatusMsg.servo = false;
		ROS_ERROR("%s",ex.what());
		ROS_ERROR("--Servo Not Validated--");
        }



	return _currentStatusMsg.servo;
}

bool MissionValidator::validateHardware(void)
{	
	_currentStatusMsg.roboteq = true;
        roboteq_node::Actuators srv;
        srv.request.actuator_position = 0;

        ROS_DEBUG("-- Trying to set actuators to 0 --");
        if(_actuatorClient.call(srv))  // blocking call
        {
		_currentStatusMsg.roboteq = true;
                ROS_DEBUG("-- Roboteq Validated --");
        }
        else
        {
		_currentStatusMsg.roboteq = false;
                ROS_ERROR_STREAM(" -- Roboteq In-Validated --");
        }

	return _currentStatusMsg.roboteq;
}

bool MissionValidator::validationServiceCallback(ValidationRequest& request, ValidationResponse& response)
{
	ROS_WARN("--Validating Sensors...--");
	_currentStatusMsg.aruco 	= false;
	_currentStatusMsg.roboteq 	= false;
	_currentStatusMsg.servo 	= false;
	_currentStatusMsg.rtab 		= false; // when the service gets called, start with a 
	_currentStatusMsg.imu 		= false; // clean sheet since nothing has been validated at this point yet
	_currentStatusMsg.camera 	= false;
	_currentStatusMsg.kinect 	= false;

	validateSensors();

	response.status = _currentStatusMsg;
	response.validated = (_eStatus == eValidated);
}

void MissionValidator::arucoPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg)
{
	// no need to have seperate validation function, we just care that an aruco pose exists/is being produced
	// NOTE: Even though we set all status msgs to false when the service gets called, there are a few ros::spinOnce() called
	// inbetween the other validation functions, so when that happens, this callback will get fired if there is a aruco msg
        ROS_INFO_STREAM_ONCE("Aruco Found: X[" << poseMsg->pose.pose.position.x << "] Y[" << poseMsg->pose.pose.position.y << "]");
	if(!_currentStatusMsg.aruco)
		_currentStatusMsg.aruco = true;
}

