#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(2);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "jackal";
  collision_objects[0].header.frame_id = "base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[1] = 0.508;
  collision_objects[0].primitives[0].dimensions[0] = 0.43;
  collision_objects[0].primitives[0].dimensions[2] = 0.25;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = -0.25/2;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;


  collision_objects[1].id = "ouster";
  collision_objects[1].header.frame_id = "base_link";


  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.087;
  collision_objects[1].primitives[0].dimensions[1] = 0.087;
  collision_objects[1].primitives[0].dimensions[2] = 0.0735;

 
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.1;
  collision_objects[1].primitive_poses[0].position.z = 0.0735/2;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD; 

  planning_scene_interface.applyCollisionObjects(collision_objects);
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gen3_add_obstacle");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("arm");
  group.setPlanningTime(45.0);

  addCollisionObjects(planning_scene_interface);

  ros::waitForShutdown();
  return 0;
}