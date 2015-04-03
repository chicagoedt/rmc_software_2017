#include "state_machine.h"

StateMachineBase::StateMachineBase(void):
	_moveBaseAC("move_base", true), _nh("state_machine"),
   MAX_GOAL_POINTS(12)
{

}

StateMachineBase::~StateMachineBase(void)
{

}

bool StateMachineBase::Initialize()
{
  std::string   goalParamName = "GoalPoint";

  for(int i = 0; i < MAX_GOAL_POINTS; i++)
  {
    std::ostringstream gp; gp << "GoalPoint" << i;

    ROS_DEBUG_STREAM("Loaded param: " << gp.str());

    if (_nh.hasParam(gp.str()))
    {
      std::vector<double>   goalXY;

      geometry_msgs::Pose   tmpPose;

      _nh.getParam(gp.str(), goalXY);

      tmpPose.position.x  = goalXY[0];
      tmpPose.position.y  = goalXY[1];
      //tf::quaternionTFToMsg(tf::createIdentityQuaternion(), tmpPose.orientation);
      //tmpPose.orientation = tf::createQuaternionMsgYaw(3.14159);

      _goalPointsQueue.push(tmpPose);
    }
    else
    {
      ROS_ERROR_STREAM("Please fill the goalpoints.yaml for GoalPoint" << i);
      return false;
    }
  }

  return true;
}


void StateMachineBase::run()
{
	while(!_moveBaseAC.waitForServer(ros::Duration(2.0)))
	{
    ROS_INFO("Waiting for the move_base action server to come up");
	}

	ROS_INFO("Established Connection with move_base ActionServer.");

  while(!_goalPointsQueue.empty())
  {
    _moveBaseGoal.target_pose.header.frame_id   = "odom";
    _moveBaseGoal.target_pose.header.stamp      = ros::Time::now();

    _moveBaseGoal.target_pose.pose.position     = _goalPointsQueue.front().position;
    _moveBaseGoal.target_pose.pose.orientation  = tf::createQuaternionMsgFromYaw(0.0);

    ROS_INFO_STREAM("Sending goal(X, Y):" << "[ " << _moveBaseGoal.target_pose.pose.position.x << " , "
                                                  << _moveBaseGoal.target_pose.pose.position.y << " ]");

    _moveBaseAC.sendGoal(_moveBaseGoal);

    _moveBaseAC.waitForResult();

    if(_moveBaseAC.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      _goalPointsQueue.pop();

      ROS_INFO("Succesfully moved to GoalPoint.");
    }
    else
    {
      ROS_INFO("Failed to move to GoalPoint.");
    }
  }



  //_moveBaseGoal.target_pose.pose.position.x = 7.0;
  //_moveBaseGoal.target_pose.pose.orientation.w = 1.0;

  //_moveBaseGoal.target_pose = _goalPoints.at(0);


}
