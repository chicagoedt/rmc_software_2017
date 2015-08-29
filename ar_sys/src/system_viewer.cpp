/**
* @file system_viewer.cpp
* @author Hamdi Sahloul
* @date September 2014
* @version 0.1
* @brief Display all the detected boards and all the cameras in a single 3D coordinates system.
*/

#include <opencv2/core/core.hpp>
#include <aruco/aruco.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#define VISUAL_MARKER_SIZE	0.08

enum position_t
{
	FIXED,
	RELATIVE,
	FLOATING,

	POSITION_NR_ITEMS
};

struct board_t
{
	int uid;
	std::string name;
	position_t type;

	std::string relativeName;
	tf::Transform transform;
};

struct relative_board_t
{
	board_t board;
	std::map<std::string, tf::Transform> transforms_map;
};

struct camera_t
{
	int uid;
	std::string name;
	position_t type;

	tf::Transform transform;
};

class ArSysViewer
{
	private:
		std::string map_path;
		std::map<std::string, board_t> boards_map;
		std::map<std::string, relative_board_t> relativeBoards_map;
		std::map<std::string, camera_t> cameras_map;
		double digital_filter_change_rate;

		ros::Subscriber transform_sub;
		ros::Publisher transform_pub; 
		ros::Publisher rviz_marker_pub;
		tf::TransformBroadcaster broadcaster;
		ros::NodeHandle nh;

	public:
		ArSysViewer()
			: nh("~")
		{
			transform_sub = nh.subscribe ("/transform", 1, &ArSysViewer::transform_callback, this);

			transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
			rviz_marker_pub = nh.advertise <visualization_msgs::Marker> ("visualization_marker", 0);

			nh.param<std::string>("map_path", map_path, "map.yml");
			nh.param<double>("digital_filter_change_rate", digital_filter_change_rate, 0.5);

			readFromFile(map_path.c_str());
			ROS_INFO("ArSys node started with boards map: %s", map_path.c_str());
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
				throw	cv::Exception ( 81818,"ArSysViewer::readFromFile",ex.what()+string(" file=)")+sfile ,__FILE__,__LINE__ );
			}
		}


		void readFromFile ( cv::FileStorage &fs ) throw ( cv::Exception )
		{
			//look for the ar_map_boards and ar_map_cameras
			if (fs["ar_map_boards"].name() != "ar_map_boards" || fs["ar_map_cameras"].name() != "ar_map_cameras")
				throw cv::Exception ( 81818,"ArSysViewer::readFromFile","invalid file type" ,__FILE__,__LINE__ );

			cv::FileNode FnBoards=fs["ar_map_boards"];
			for (cv::FileNodeIterator it = FnBoards.begin(); it != FnBoards.end(); ++it)
			{
				board_t board;

				board.uid = boards_map.size();
				board.name = (std::string)(*it)["name"];
				if ((std::string)(*it)["type"] == "fixed")
				{
					board.type = FIXED;

				}
				else if ((std::string)(*it)["type"] == "relative")
				{
					board.type = RELATIVE;
					board.relativeName = (std::string)(*it)["relativeName"];
				}
				else
				{
					if ((std::string)(*it)["type"] != "floating") ROS_WARN("Unknow board type: %s for board %s, assuming floating",
						((std::string)(*it)["type"]).c_str(),
						board.name.c_str());
					board.type = FLOATING;
					board.transform = tf::Transform::getIdentity(); //Initial position
				}
				if (board.type == FIXED || board.type == RELATIVE)
				{
					vector<float> coordinates3d;
					cv::FileNode FnPose = (*it)["pose"];
					cv::FileNodeIterator itc = FnPose.begin();

					(*itc)>>coordinates3d;
					if (coordinates3d.size() != 3)
				    		throw cv::Exception ( 81818,"ArSysViewer::readFromFile","invalid file type 3" ,__FILE__,__LINE__ );
					tf::Vector3 ref_origin(coordinates3d[0], coordinates3d[1], coordinates3d[2]);

					(*++itc)>>coordinates3d;
					if (coordinates3d.size() != 3)
				    		throw cv::Exception ( 81818,"ArSysViewer::readFromFile","invalid file type 3" ,__FILE__,__LINE__ );
					tf::Quaternion ref_rotation = tf::createQuaternionFromRPY(coordinates3d[0], coordinates3d[1], coordinates3d[2]);

					board.transform = tf::Transform(ref_rotation, ref_origin);

					if (board.type == RELATIVE)
					{
						if (relativeBoards_map.count(board.relativeName) == 0)
						{
							relative_board_t relativeBoard;

							relativeBoard.board.uid = relativeBoards_map.size();
							relativeBoard.board.name = board.relativeName;
							relativeBoard.board.type = FLOATING;
							relativeBoard.board.transform = tf::Transform::getIdentity(); //Initial position

							relativeBoards_map.insert(std::make_pair(relativeBoard.board.name, relativeBoard));
						}
						relativeBoards_map[board.relativeName].transforms_map.insert(std::make_pair(board.name, board.transform));
						board.transform = tf::Transform::getIdentity(); //Done with relative, Initial position
					}
				}

				boards_map.insert(std::make_pair(board.name, board));
			}

			cv::FileNode FnCameras=fs["ar_map_cameras"];
			for (cv::FileNodeIterator it = FnCameras.begin(); it != FnCameras.end(); ++it)
			{
				camera_t camera;

				camera.uid = cameras_map.size();
				camera.name = (std::string)(*it)["name"];
				if ((std::string)(*it)["type"] == "fixed")
				{
					camera.type = FIXED;

					vector<float> coordinates3d;
					cv::FileNode FnPose = (*it)["pose"];
					cv::FileNodeIterator itc = FnPose.begin();

					(*itc)>>coordinates3d;
					if (coordinates3d.size() != 3)
				    		throw cv::Exception ( 81818,"ArSysViewer::readFromFile","invalid file type 3" ,__FILE__,__LINE__ );
					tf::Vector3 ref_origin(coordinates3d[0], coordinates3d[1], coordinates3d[2]);

					(*++itc)>>coordinates3d;
					if (coordinates3d.size() != 3)
				    		throw cv::Exception ( 81818,"ArSysViewer::readFromFile","invalid file type 3" ,__FILE__,__LINE__ );
					tf::Quaternion ref_rotation = tf::createQuaternionFromRPY(coordinates3d[0], coordinates3d[1], coordinates3d[2]);

					camera.transform = tf::Transform(ref_rotation, ref_origin);
				}
				else
				{
					if ((std::string)(*it)["type"] != "floating") ROS_WARN("Unknow camera type: %s for camera %s, assuming floating",
						((std::string)(*it)["type"]).c_str(),
						camera.name.c_str());
					camera.type = FLOATING;
					camera.transform = tf::Transform::getIdentity(); //Initial position
				}

				cameras_map.insert(std::make_pair(camera.name, camera));
			}
		}

		tf::Transform& digital_filter(tf::Transform &dst, const tf::Transform &src)
		{
			//dst = src;

			tf::Vector3 posOld = dst.getOrigin();
			tf::Vector3 posNew = src.getOrigin();
			tf::Vector3 pos
			(
				(1 - digital_filter_change_rate) * posOld.x() + digital_filter_change_rate * posNew.x(),
				(1 - digital_filter_change_rate) * posOld.y() + digital_filter_change_rate * posNew.y(),
				(1 - digital_filter_change_rate) * posOld.z() + digital_filter_change_rate * posNew.z()
			);
			dst.setOrigin(pos);

			tf::Quaternion ornOld = dst.getRotation();
			tf::Quaternion ornNew = src.getRotation();
			tf::Quaternion orn
			(
				(1 - digital_filter_change_rate) * ornOld.x() + digital_filter_change_rate * ornNew.x(),
				(1 - digital_filter_change_rate) * ornOld.y() + digital_filter_change_rate * ornNew.y(),
				(1 - digital_filter_change_rate) * ornOld.z() + digital_filter_change_rate * ornNew.z(),
				(1 - digital_filter_change_rate) * ornOld.w() + digital_filter_change_rate * ornNew.w()
			);
			dst.setRotation(orn);

			return dst;
		}

		void transform_callback (const geometry_msgs::TransformStamped& transformMsg)
		{
			if (cameras_map.count(transformMsg.header.frame_id) == 0)
			{
				ROS_WARN("Map file does not contain an entry for the '%s' camera! Adding as floating", transformMsg.header.frame_id.c_str());

				camera_t camera;
				camera.uid = cameras_map.size();
				camera.name = transformMsg.header.frame_id;
				camera.type = FLOATING;
				camera.transform = tf::Transform::getIdentity(); //Initial position
				cameras_map.insert(std::make_pair(camera.name, camera));
			}
			if (boards_map.count(transformMsg.child_frame_id) == 0)
			{
				ROS_WARN("Map file does not contain an entry for the '%s' board! Adding as floating", transformMsg.child_frame_id.c_str());

				board_t board;
				board.uid = boards_map.size();
				board.name = transformMsg.child_frame_id;
				board.type = FLOATING;
				board.transform = tf::Transform::getIdentity(); //Initial position
				boards_map.insert(std::make_pair(board.name, board));
			}

			tf::StampedTransform stampedTransform;
			tf::transformStampedMsgToTF(transformMsg, stampedTransform);
			if (cameras_map[transformMsg.header.frame_id].type == FLOATING && boards_map[transformMsg.child_frame_id].type == FIXED)
			{
				digital_filter(cameras_map[transformMsg.header.frame_id].transform, boards_map[transformMsg.child_frame_id].transform * stampedTransform.inverse());
			}
			tf::StampedTransform camStampedTransform (cameras_map[transformMsg.header.frame_id].transform, transformMsg.header.stamp, "world", transformMsg.header.frame_id);
			broadcaster.sendTransform(camStampedTransform);

			stampedTransform.setData(boards_map[transformMsg.child_frame_id].type == FIXED ?
				boards_map[transformMsg.child_frame_id].transform :
				digital_filter(boards_map[transformMsg.child_frame_id].transform, cameras_map[transformMsg.header.frame_id].transform * stampedTransform));
			stampedTransform.frame_id_ = camStampedTransform.frame_id_;
			broadcaster.sendTransform(stampedTransform);
			geometry_msgs::TransformStamped absTransformMsg;
			tf::transformStampedTFToMsg(stampedTransform, absTransformMsg);
			transform_pub.publish(absTransformMsg);
			if (boards_map[transformMsg.child_frame_id].type == RELATIVE)
			{
				std::string relativeName = boards_map[transformMsg.child_frame_id].relativeName;

				digital_filter(relativeBoards_map[relativeName].board.transform, boards_map[transformMsg.child_frame_id].transform * relativeBoards_map[relativeName].transforms_map[transformMsg.child_frame_id]);
				tf::StampedTransform relativeStampedTransform(relativeBoards_map[relativeName].board.transform, transformMsg.header.stamp, "world", relativeName);
				broadcaster.sendTransform(relativeStampedTransform);
				tf::transformStampedTFToMsg(relativeStampedTransform, absTransformMsg);
				transform_pub.publish(absTransformMsg);
			}

			//Generate visual marker
			tf::Vector3 markerOrigin (0, 0, 0.1 * VISUAL_MARKER_SIZE);
			tf::Transform m (tf::Quaternion::getIdentity(), markerOrigin);
			tf::Transform markerPose = boards_map[transformMsg.child_frame_id].transform * m;

			visualization_msgs::Marker visualMarker;
			tf::poseTFToMsg (markerPose, visualMarker.pose);

			visualMarker.header.frame_id = stampedTransform.frame_id_;
			visualMarker.header.stamp = transformMsg.header.stamp;
			visualMarker.id = boards_map[transformMsg.child_frame_id].uid;

			visualMarker.scale.x = 1.0 * VISUAL_MARKER_SIZE;
			visualMarker.scale.y = 1.0 * VISUAL_MARKER_SIZE;
			visualMarker.scale.z = 0.1 * VISUAL_MARKER_SIZE;
			visualMarker.ns = "basic_shapes";
			visualMarker.type = visualization_msgs::Marker::CUBE;
			visualMarker.action = visualization_msgs::Marker::ADD;
			visualMarker.color.r = (float)visualMarker.id / (boards_map.size() - 1);
			visualMarker.color.g = 1.0 - visualMarker.color.r;
			visualMarker.color.b = (float)boards_map[transformMsg.child_frame_id].type / (POSITION_NR_ITEMS - 1);
			visualMarker.color.a = 1.0;
			visualMarker.lifetime = ros::Duration (1.0);
			//Publish visual marker
			rviz_marker_pub.publish(visualMarker);
		}
};


int main(int argc,char **argv)
{
	ros::init(argc, argv, "ar_system_viewer");

	ArSysViewer node;

	ros::spin();
}
