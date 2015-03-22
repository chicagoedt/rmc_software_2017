#include "state_machine.h"

StateMachineBase::StateMachineBase(void):
	_moveBaseAC("move_base", true), _nh("state_machine"),
   MAX_GOAL_POINTS(5)
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

    ROS_INFO_STREAM(gp.str());

    if (_nh.hasParam(gp.str()))
    {
      std::vector<double>   goalXY;

      geometry_msgs::Pose   tmpPose;

      _nh.getParam(gp.str(), goalXY);

      tmpPose.position.x = goalXY[0];
      tmpPose.position.y = goalXY[1];

      _goalPoints.push_back(tmpPose);
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

	_moveBaseGoal.target_pose.header.frame_id = "odom";
  _moveBaseGoal.target_pose.header.stamp = ros::Time::now();

  //_moveBaseGoal.target_pose.pose.position.x = 7.0;
  //_moveBaseGoal.target_pose.pose.orientation.w = 1.0;

  //_moveBaseGoal.target_pose = _goalPoints.at(0);

  ROS_INFO("Sending goal");
  _moveBaseAC.sendGoal(_moveBaseGoal);

  _moveBaseAC.waitForResult();

  if(_moveBaseAC.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  ROS_INFO("Hooray, the base moved 1 meter forward");
  else
  ROS_INFO("The base failed to move forward 1 meter for some reason");
}
