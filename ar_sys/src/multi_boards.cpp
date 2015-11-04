/**
* @file multi_boards.cpp
* @author Hamdi Sahloul
* @date September 2014
* @version 0.1
* @brief Detect multi-boards simultaneously.
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <aruco/aruco.h>
#include <aruco/boarddetector.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ar_sys/utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
//#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace aruco;

struct board_t
{
	int uid;
	std::string name;
	BoardConfiguration config;
	double marker_size;
};

class ArSysMultiBoards
{
	private:
		cv::Mat inImage, resultImg;
		aruco::CameraParameters camParam;
		bool useRectifiedImages;
		bool draw_markers;
		bool draw_markers_cube;
		bool draw_markers_axis;
		MarkerDetector mDetector;
		vector<Marker> markers;
		BoardDetector the_board_detector;
		ros::Subscriber cam_info_sub;
		bool cam_info_received;
		image_transport::Publisher image_pub;
		image_transport::Publisher debug_pub;
		ros::Publisher pose_pub;
		ros::Publisher transform_pub; 
		ros::Publisher position_pub;
		std::string boards_config;
		std::string boards_directory;
		vector<board_t> boards;

		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Subscriber image_sub;


		tf::TransformBroadcaster broadcaster;
		tf::TransformListener _tfListener;

		//tf2_ros::Buffer tfBuffer;
		//tf2_ros::TransformListener tfListener;

		//geometry_msgs::TransformStamped imOffsetTransform;
		tf::StampedTransform imOffsetTransform;
		tf::StampedTransform rotateAruco;
		//tf::Stamped<geometry_msgs::Pose> imOffsetTransform;

	public:
		ArSysMultiBoards()
			: cam_info_received(false),
			nh("~"),
			it(nh)
		{
			image_sub = it.subscribe("/image", 1, &ArSysMultiBoards::image_callback, this);
			cam_info_sub = nh.subscribe("/camera_info", 1, &ArSysMultiBoards::cam_info_callback, this);

			image_pub = it.advertise("result", 1);
			debug_pub = it.advertise("debug", 1);
			pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 100);
			transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
			position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);

			nh.param<std::string>("boards_config", boards_config, "boardsConfiguration.yml");
			nh.param<std::string>("boards_directory", boards_directory, "./data");
			nh.param<bool>("image_is_rectified", useRectifiedImages, true);
			nh.param<bool>("draw_markers", draw_markers, false);
			nh.param<bool>("draw_markers_cube", draw_markers_cube, false);
			nh.param<bool>("draw_markers_axis", draw_markers_axis, false);

			readFromFile(boards_config.c_str());
			ROS_INFO("ArSys node started with boards configuration: %s", boards_config.c_str());
		}

		void readFromFile ( string sfile ) throw ( cv::Exception )
		{
			try
			{
				cv::FileStorage fs ( sfile,cv::FileStorage::READ );
				readFromFile ( fs );
			}
			catch (std::exception &ex)
			{
				throw	cv::Exception ( 81818,"ArSysMultiBoards::readFromFile",ex.what()+string(" file=)")+sfile ,__FILE__,__LINE__ );
			}
		}


		void readFromFile ( cv::FileStorage &fs ) throw ( cv::Exception )
		{
			//look for the ar_sys_boards
			if (fs["ar_sys_boards"].name() != "ar_sys_boards")
				throw cv::Exception ( 81818,"ArSysMultiBoards::readFromFile","invalid file type" ,__FILE__,__LINE__ );

			cv::FileNode FnBoards=fs["ar_sys_boards"];
			for (cv::FileNodeIterator it = FnBoards.begin(); it != FnBoards.end(); ++it)
			{
				board_t board;

				board.uid = boards.size();
				board.name = (std::string)(*it)["name"];
				board.marker_size = (double)(*it)["marker_size"];

				std::string path(boards_directory);
				path.append("/");
				path.append((std::string)(*it)["path"]);
				board.config.readFromFile(path);

				boards.push_back(board);
			}

			ROS_ASSERT(boards.size() > 0);
		}

		void image_callback(const sensor_msgs::ImageConstPtr& msg)
		{
			if(!cam_info_received) return;

			cv_bridge::CvImagePtr cv_ptr;
			try
			{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
				inImage = cv_ptr->image;
				resultImg = cv_ptr->image.clone();

				//detection results will go into "markers"
				markers.clear();

				//Ok, let's detect
				double min_size = boards[0].marker_size;
				for (int board_index = 1; board_index < boards.size(); board_index++)
					if (min_size > boards[board_index].marker_size) min_size = boards[board_index].marker_size;
				mDetector.detect(inImage, markers, camParam, min_size, false);


				try
				{
					//_tfListener.waitForTransform("base_link", "blackfly_optical_link", ros::Time(0), ros::Duration(1.0));
					//imOffsetTransform = tfBuffer.lookupTransform("blackfly_optical_link", "base_link", ros::Time(0));
					_tfListener.lookupTransform("base_link", "blackfly_mount_link", ros::Time(0), imOffsetTransform);
					//_tfListener.lookupTransform("board_frame", "map", ros::Time(0), rotateAruco);

					for (int board_index = 0; board_index < boards.size(); board_index++)
					{
						Board board_detected;

						//Detection of the board
						float probDetect = the_board_detector.detect(markers, boards[board_index].config, board_detected, camParam, boards[board_index].marker_size);
						if (probDetect > 0.0 && board_index == 2)
						{ 

							tf::Transform transform = ar_sys::getTf(board_detected.Rvec, board_detected.Tvec);



							transform *= imOffsetTransform.inverse();

							tf::StampedTransform stampedTransform(transform, ros::Time::now(), "board_marker", "base_link");


							geometry_msgs::PoseStamped rawPoseMsg;
							rawPoseMsg.header.frame_id = "board_marker";
							rawPoseMsg.header.stamp = msg->header.stamp;

							geometry_msgs::PoseStamped newPoseMsg;
							geometry_msgs::PoseWithCovarianceStamped poseMsg;

							tf::poseTFToMsg(transform, rawPoseMsg.pose);

							_tfListener.transformPose("map", rawPoseMsg, newPoseMsg);

							poseMsg.pose.pose = newPoseMsg.pose;

							poseMsg.pose.covariance[0] = 0.005;
							poseMsg.pose.covariance[7] = 0.005;
							poseMsg.pose.covariance[14] = 0.005;
							poseMsg.pose.covariance[21] = 0.006;
							poseMsg.pose.covariance[28] = 0.006;
							poseMsg.pose.covariance[35] = 0.006;
							poseMsg.header.frame_id = "map";
							poseMsg.header.stamp = msg->header.stamp;
							pose_pub.publish(poseMsg);

							//broadcaster.sendTransform(stampedTransform);

							geometry_msgs::TransformStamped transformMsg;
							tf::transformStampedTFToMsg(stampedTransform, transformMsg);
							transform_pub.publish(transformMsg);

							geometry_msgs::Vector3Stamped positionMsg;
							positionMsg.header = transformMsg.header;
							positionMsg.vector = transformMsg.transform.translation;
							position_pub.publish(positionMsg);

							if(camParam.isValid())
							{
								//draw board axis
								CvDrawingUtils::draw3dAxis(resultImg, board_detected, camParam);
							}
						}
					}
				}
				catch (tf2::TransformException &ex) 
				{
					ROS_ERROR("%s",ex.what());
				}

				//for each marker, draw info and its boundaries in the image
				for(size_t i=0; draw_markers && i < markers.size(); ++i)
				{
					markers[i].draw(resultImg,cv::Scalar(0,0,255),2);
				}

				if(camParam.isValid())
				{
					//draw a 3d cube in each marker if there is 3d info
					for(size_t i=0; i<markers.size(); ++i)
					{
						if (draw_markers_cube) CvDrawingUtils::draw3dCube(resultImg, markers[i], camParam);
						if (draw_markers_axis) CvDrawingUtils::draw3dAxis(resultImg, markers[i], camParam);
					}
				}

				if(image_pub.getNumSubscribers() > 0)
				{
					//show input with augmented information
					cv_bridge::CvImage out_msg;
					out_msg.header.frame_id = msg->header.frame_id;
					out_msg.header.stamp = msg->header.stamp;
					out_msg.encoding = sensor_msgs::image_encodings::RGB8;
					out_msg.image = resultImg;
					image_pub.publish(out_msg.toImageMsg());
				}

				if(debug_pub.getNumSubscribers() > 0)
				{
					//show also the internal image resulting from the threshold operation
					cv_bridge::CvImage debug_msg;
					debug_msg.header.frame_id = msg->header.frame_id;
					debug_msg.header.stamp = msg->header.stamp;
					debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
					debug_msg.image = mDetector.getThresholdedImage();
					debug_pub.publish(debug_msg.toImageMsg());
				}
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
		}

		// wait for one camerainfo, then shut down that subscriber
		void cam_info_callback(const sensor_msgs::CameraInfo &msg)
		{
			camParam = ar_sys::getCamParams(msg, useRectifiedImages);
			cam_info_received = true;
			cam_info_sub.shutdown();
		}
};


int main(int argc,char **argv)
{
	ros::init(argc, argv, "ar_multi_boards");

	ArSysMultiBoards node;

	ros::spin();
}
