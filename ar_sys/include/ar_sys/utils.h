#ifndef UTILS_H
#define UTILS_H

#include "aruco/aruco.h"
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_datatypes.h>

namespace ar_sys
{
  /**
     * @brief getCamParams gets the camera intrinsics from a CameraInfo message and copies them
     *                                     to ar_sys own data structure
     * @param cam_info
     * @param useRectifiedParameters if true, the intrinsics are taken from cam_info.P and the distortion parameters
     *                               are set to 0. Otherwise, cam_info.K and cam_info.D are taken.
     * @return
     */
  aruco::CameraParameters getCamParams(const sensor_msgs::CameraInfo& cam_info,
                                                       bool useRectifiedParameters);
  /**
     * @brief getTf converts OpenCV coordinates to ROS Transform
     * @param Rvec
     * @param Tvec
     * @return tf::Transform
     */
  tf::Transform getTf(const cv::Mat &Rvec, const cv::Mat &Tvec);

}
#endif // UTILS_H
