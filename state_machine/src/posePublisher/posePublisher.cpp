#include "state_machine.h"
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

//#include "../../include/serialPort.h"

#include "../../../rmcDecode/rmcEnDecoder.h"
#include "serialPort.h"

class RosLogger : public oxoocoffee::SerialLogger
{
    public:
        virtual bool    IsLogOpen(void) const { return true; }
        virtual void    LogLine(const char* pBuffer, unsigned int len)
        {
            std::cout.write(pBuffer, len) << std::endl;
        }

        virtual void    LogLine(const std::string& message)
        {
            std::cout << message << std::endl;
        }

        // DO NOT Write new line at end
        virtual void    Log(const char* pBuffer, unsigned int len)
        {
            std::cout.write(pBuffer, len);
        }

        virtual void    Log(const std::string& message)
        {
            std::cout << message;
        }
};

void stateCallback(const std_msgs::Int16::ConstPtr& msg){
	
}

bool createData(double &x, double &y, double &yaw){
	// TODO: PRESCALING
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_publisher_node");
	ros::NodeHandle nh;
	ros::Publisher posePub;
	ros::Subscriber stateSub;
	tf::TransformListener tf_listener;
	double x = 0, y = 0, yaw = 0;
	bool is_test = true;

	string device;
	string defaultstr="/dev/ttyACM0";
	nh.param("sigmuxSerial", device, defaultstr);

	stateSub = nh.subscribe("dig_state", 1, &stateCallback);

	tf::StampedTransform transform;

	RMCEnDecoder decoder;
	RosLogger logger;
	oxoocoffee::SerialPort port(logger);

	ros::Rate loop_rate(4);
	ros::spinOnce();
	loop_rate.sleep();

	port.baud(115200);
	port.connect(device);

	if (!is_test) while (ros::ok()){
		try
		{
			tf_listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
		}
		catch (tf::TransformException ex) {
			try {
				tf_listener.lookupTransform("/ar_board_marker", "/base_link", ros::Time(0), transform);
			}
			catch (tf::TransformException ex2) {
				ros::spinOnce();
				loop_rate.sleep();
				// print error?
				continue;
			}
		}
		// GET DATA
		x = transform.getOrigin().getX();
		y = transform.getOrigin().getY();
		double roll, pitch;
		tf::Quaternion rotation = transform.getRotation();
		tf::Matrix3x3(rotation).getRPY(roll, pitch, yaw);

		// Normalize Data

		x = static_cast <int> (x*100);
		y = static_cast <int> (y*100);
		y += 1.89;
		yaw = static_cast <int> ((yaw / 31) * 180/M_PI);
		if (yaw < 0) {yaw = 360 + yaw;}

		// check for errors

		if (x < 0 || x > 730) {/*some error crap*/}
		if (y < 0 || y > 500) {/*some error crap*/}

		ROS_INFO_STREAM("x, y, z");
		ROS_INFO_STREAM(x);
		ROS_INFO_STREAM(y);
		ROS_INFO_STREAM(yaw);

		RMCData data(x, y, yaw, RMCData::eDigState_Home);

		const RMCEnDecoder::TVec& vector = decoder.encodeMessage(data);

		// serial stuff

		

	}
	else {
		while (true) {
			yaw = 0;
			int i = 0;
			for (i = 0; i <= 378; i++){
				x = static_cast <int> (i);
				y = static_cast <int> (i);
				yaw = static_cast <int> (yaw);
				RMCData data(y, x, yaw, RMCData::eDigState_Home);
				const RMCEnDecoder::TVec& vector = decoder.encodeMessage(data);

				const char* buffer = reinterpret_cast <const char*> (vector.buffer());
				port.write(buffer, vector.size());

				ROS_INFO_STREAM("x, y, z");
				ROS_INFO_STREAM(i);
				ROS_INFO_STREAM(i);
				ROS_INFO_STREAM(yaw);
				ros::Duration(.25).sleep();
			}
			for (; i >= 0; i--) {
				x = static_cast <int> (i);
				y = static_cast <int> (i);
				yaw = static_cast <int> (yaw);
				RMCData data(y, x, yaw, RMCData::eDigState_Home);
				const RMCEnDecoder::TVec& vector = decoder.encodeMessage(data);

				const char* buffer = reinterpret_cast <const char*> (vector.buffer());
				port.write(buffer, vector.size());

				ROS_INFO_STREAM("x, y, z");
				ROS_INFO_STREAM(i);
				ROS_INFO_STREAM(i);
				ROS_INFO_STREAM(yaw);
				ros::Duration(.25).sleep();
				
			}
		}
	}

    return 0;
}

