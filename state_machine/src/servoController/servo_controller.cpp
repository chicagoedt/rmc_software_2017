#include "servo_controller.h"

ServoController::ServoController(): _gotArucoPose(false)
{

}

ServoController::~ServoController()
{

}

bool ServoController::Initialize()
{
	_servoPosePub = _nh.advertise<std_msgs::Float64>("blackfly_mount_joint/command", 1);
	_arucoSub = _nh.subscribe("ar_single_board/pose", 1, &ServoController::arucoPoseCallback, this);
/*
	std_msgs::Float64 startYaw;
	startYaw.data = 0;

	_servoPosePub.publish(startYaw); // Start the servo at 0 radians

	ros::spinOnce();

	ros::Duration hold(5);
	hold.sleep();
*/
	return true;
}

void ServoController::arucoPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg)
{
	_gotArucoPose = true;
}

void ServoController::zeroServo()
{
	std_msgs::Float64 servoYawRadians;
	servoYawRadians.data = 0;
	bool isZero = false;
	ros::Rate rate(10);

	tf::StampedTransform servoTf;

	while(!isZero)
	{
		ROS_INFO("Looping");
                        try
                        {
                                _tfListener.waitForTransform("servo_mount_link","blackfly_mount_link", ros::Time(0), ros::Duration(3));
                                _tfListener.lookupTransform("servo_mount_link","blackfly_mount_link", ros::Time(0), servoTf);
                        }
                        catch (tf::TransformException &ex)
                        {
                                ROS_ERROR("%s",ex.what());
                                ros::Duration(1.0).sleep();
                                continue;
               		}

		if(getYaw(servoTf.getRotation()) < 0.09 && getYaw(servoTf.getRotation()) > -0.09)
		{
			isZero = true;
			ROS_INFO("ZERO!");
		}
		else
		{
			_servoPosePub.publish(servoYawRadians);
		}
		ros::spinOnce();
		rate.sleep();

	}
}

void ServoController::run()
{
	bool rotateCCW_done = false;
	std_msgs::Float64 servoYawRadians;	

	tf::StampedTransform servoTf;

	ROS_INFO("Running!!");

	ros::Rate rate(10.0);

	bool isZeroed = false;

	//zeroServo();
	
	while(ros::ok())
	{
		if(!_gotArucoPose)
		{

			try
			{
				_tfListener.waitForTransform("servo_mount_link","blackfly_mount_link", ros::Time(0), ros::Duration(3));
				_tfListener.lookupTransform("servo_mount_link","blackfly_mount_link", ros::Time(0), servoTf);
			}
			catch (tf::TransformException &ex) 
			{
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
				continue;
			}

			double yaw = getYaw(servoTf.getRotation());
			ROS_INFO_STREAM("Yaw is: " << yaw);

			if(!rotateCCW_done)
			{
				ROS_INFO("CCW");
				servoYawRadians.data = 2.62; // 150 degrees

				if(yaw <= -0.55)
					rotateCCW_done = true;
			}
			else
			{
				ROS_INFO("CW");
				servoYawRadians.data = 0; // -150 degrees
			}
			
			_servoPosePub.publish(servoYawRadians);
		}

		ros::spinOnce();
		rate.sleep();
	}
}
