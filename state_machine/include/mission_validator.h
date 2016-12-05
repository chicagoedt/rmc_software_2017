#ifndef Mission_Validator_h
#define Mission_Validator_h

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <roboteq_node/Actuators.h>
#include <state_machine/SensorStatus.h>
#include <state_machine/ValidateSensors.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// TODO
// create service to check sensors during mid runtime
// check on the way back from digging if aruco can be found by some distance threshold
// if moving forward and rtabmap doesnt pick up where it left off, call rtabmap reset service

typedef state_machine::ValidateSensors::Request  ValidationRequest;
typedef state_machine::ValidateSensors::Response ValidationResponse;

class MissionValidator
{
	public:
		MissionValidator(void);
		~MissionValidator(void);

		void Initialize(void);
		void Run(void);

	private:
		bool validateSensors(ValidationRequest request);
		bool validateRtab(void);
		bool validateServo(void);
		bool validateArucoTF(void);
		bool validateHardware(void);

		void arucoPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg);

		bool validationServiceCallback(ValidationRequest& request, ValidationResponse& response);
		
	private:
		enum eValidationStatus
		{
			eValidated,
			eInvalidated,
			eEvaluating
		};
		
		ros::NodeHandle _nh;

		ros::Subscriber _arucoSub;
		ros::Subscriber _imuSub;

		ros::Publisher	_servoPub;

		// if service request is made, and all sensors valid, response should be left empty to avoid unnecessary overhead
		// if not all sensors are valid, fill response with sensor status msg
		ros::ServiceServer  		_service;
		ros::ServiceClient		_actuatorClient;

		ros::Rate			_loopRate;

		tf::TransformListener   	_tfListener;

		state_machine::SensorStatus 	_currentStatusMsg;

		eValidationStatus 		_eStatus;		

		bool _rtabValidated;
		bool _arucoValidated;


};

#endif
