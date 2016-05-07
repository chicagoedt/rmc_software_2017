#ifndef State_Machine_h
#define State_Machine_h

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <std_msgs/Float64.h>

class ServoController
{
        public:

                ServoController();
                ~ServoController();

                void run();
                bool Initialize();

        private:

                ros::NodeHandle _nh;
                ros::NodeHandle _nhLocal;

                ros::Subscriber _arucoSub;
                ros::Publisher  _servoPosePub;

		tf::TransformBroadcaster _tfBroadcaster;
		tf::TransformListener _tfListener;

		bool 		_gotArucoPose;

		void zeroServo();
                void arucoPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg);
};
#endif
