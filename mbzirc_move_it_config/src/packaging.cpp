// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;
void packaging(moveit::planning_interface::MoveGroupInterface& group)
{
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup("arm");
  // Next get the current set of joint values for the group.
  moveit::core::RobotStatePtr current_state = group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);


  joint_group_positions[0] = 0;  
  joint_group_positions[1] = -30*0.0174533;
  joint_group_positions[2] = 180*0.0174533;
  joint_group_positions[3] = -146*0.0174533;
  joint_group_positions[4] = 0*0.0174533; //ok
  joint_group_positions[5] = 115*0.0174533; //ok
  joint_group_positions[6] = 270*0.0174533; //ok
  
  group.setJointValueTarget(joint_group_positions);

  ROS_WARN("Calling planner");
  bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_WARN("Planning %d", success);

  ROS_WARN("Moving Robot");

  group.move();

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "gen3_packaging");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();


  ROS_WARN("Init");

  ros::WallDuration(1.0).sleep();

  ROS_WARN("move group init1");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ROS_WARN("move group init2");
  moveit::planning_interface::MoveGroupInterface group("arm");
  group.setPlanningTime(45.0);


  ROS_WARN("move group init");

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  packaging(group);
  
  ros::waitForShutdown();

  return 0;
}


