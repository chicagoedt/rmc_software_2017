/**
* @file utils.cpp
* @author Bence Magyar
* @date June 2012
* @version 0.1
* @brief ROS2ArUco utilities.
*/

#include <ar_sys/utils.h>
#include <ros/console.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <opencv2/calib3d/calib3d.hpp>

aruco::CameraParameters ar_sys::getCamParams(const sensor_msgs::CameraInfo& cam_info,
	bool useRectifiedParameters)
{
	cv::Mat cameraMatrix(3, 3, CV_32FC1);
	cv::Mat distorsionCoeff(4, 1, CV_32FC1);
	cv::Size size(cam_info.height, cam_info.width);

	if ( useRectifiedParameters )
	{
		cameraMatrix.setTo(0);
		cameraMatrix.at<float>(0,0) = cam_info.P[0]; cameraMatrix.at<float>(0,1) = cam_info.P[1]; cameraMatrix.at<float>(0,2) = cam_info.P[2];
		cameraMatrix.at<float>(1,0) = cam_info.P[4]; cameraMatrix.at<float>(1,1) = cam_info.P[5]; cameraMatrix.at<float>(1,2) = cam_info.P[6];
		cameraMatrix.at<float>(2,0) = cam_info.P[8]; cameraMatrix.at<float>(2,1) = cam_info.P[9]; cameraMatrix.at<float>(2,2) = cam_info.P[10];

		for(int i=0; i<4; ++i)
			distorsionCoeff.at<float>(i, 0) = 0;
	}
	else
	{
		for(int i=0; i<9; ++i)
			cameraMatrix.at<float>(i%3, i-(i%3)*3) = cam_info.K[i];

		for(int i=0; i<4; ++i)
			distorsionCoeff.at<float>(i, 0) = cam_info.D[i];
	}

	return aruco::CameraParameters(cameraMatrix, distorsionCoeff, size);
}

tf::Transform ar_sys::getTf(const cv::Mat &Rvec, const cv::Mat &Tvec)
{
	cv::Mat rot(3, 3, CV_32FC1);
	cv::Rodrigues(Rvec, rot);

	cv::Mat rotate_to_sys(3, 3, CV_32FC1);
	/**
	/* Fixed the rotation to meet the ROS system
	/* Doing a basic rotation around X with theta=PI
	/* By Sahloul
	/* See http://en.wikipedia.org/wiki/Rotation_matrix for details
	*/

	//	1	0	0
	//	0	-1	0
	//	0	0	-1
	rotate_to_sys.at<float>(0,0) = 1.0;
	rotate_to_sys.at<float>(0,1) = 0.0;
	rotate_to_sys.at<float>(0,2) = 0.0;
	rotate_to_sys.at<float>(1,0) = 0.0;
	rotate_to_sys.at<float>(1,1) = -1.0;
	rotate_to_sys.at<float>(1,2) = 0.0;
	rotate_to_sys.at<float>(2,0) = 0.0;
	rotate_to_sys.at<float>(2,1) = 0.0;
	rotate_to_sys.at<float>(2,2) = -1.0;
	rot = rot*rotate_to_sys.t();

	tf::Matrix3x3 tf_rot(rot.at<float>(0,0), rot.at<float>(0,1), rot.at<float>(0,2),
		rot.at<float>(1,0), rot.at<float>(1,1), rot.at<float>(1,2),
		rot.at<float>(2,0), rot.at<float>(2,1), rot.at<float>(2,2));

	tf::Vector3 tf_orig(Tvec.at<float>(0,0), Tvec.at<float>(1,0), Tvec.at<float>(2,0));

	return tf::Transform(tf_rot, tf_orig);
}
