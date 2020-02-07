#include "ros/ros.h"
#include "mbzirc_move_it_config/SetAttachedObject.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


class MoveitInterface
{
public:
  MoveitInterface();
  ~MoveitInterface();

private:

  bool service_cb(mbzirc_move_it_config::SetAttachedObject::Request  &req,
                                 mbzirc_move_it_config::SetAttachedObject::Response &res);

  ros::NodeHandle nh;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
  
  ros::ServiceServer my_service;


  //moveit::planning_interface::MoveGroupInterface move_group("arm");
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

};

MoveitInterface::MoveitInterface()
{

 

  //static const std::string PLANNING_GROUP = "manipulator";
  static const std::string PLANNING_GROUP = "arm"; //TODO make it param

  move_group.reset(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));
  //move_group_->setPlannerId("RRTConnectkConfigDefault");

  // class to add and remove collision objects in the "virtual world" scene
  planning_scene_interface.reset(new moveit::planning_interface::PlanningSceneInterface);

  //start the service server
  my_service = nh.advertiseService("/attach_object", &MoveitInterface::service_cb, this);


}

//Destructor
MoveitInterface::~MoveitInterface()
{
}

bool MoveitInterface::service_cb(mbzirc_move_it_config::SetAttachedObject::Request  &req,
                                 mbzirc_move_it_config::SetAttachedObject::Response &res)
{


  //Define collision object and ref frame
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id ="gripper_base_link";

  // The id of the object is used to identify it.
  collision_object.id = "brick";

  //move_group->detachObject(collision_object.id);
  //std::vector<std::string> object_ids;
  //object_ids.push_back(collision_object.id);
  //planning_scene_interface->removeCollisionObjects(object_ids);
  //usleep(50000);

  if (req.attach){

    ROS_WARN("Setting Dimensions");
    // Define the dimensions of brick
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = req.length;
    primitive.dimensions[1] = req.width;
    primitive.dimensions[2] = req.height;

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose brick_pose;
    brick_pose.orientation.w = 1.0;
    brick_pose.position.x = 0;
    brick_pose.position.y = 0;
    brick_pose.position.z = (0.1524/2)+(req.height/2); //length of gripper (cylinder)

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(brick_pose);

    collision_object.operation = collision_object.APPEND; //Replace the previous exisitng obect, if none then add new.

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    ROS_WARN("Adding Object");
    planning_scene_interface->applyCollisionObjects(collision_objects); //use apply instead of add CollisionObject solves the issue of not properly adding
    ROS_WARN("Attaching Object");
    move_group->attachObject(collision_object.id);
  }

  else{

    move_group->detachObject(collision_object.id);
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    planning_scene_interface->removeCollisionObjects(object_ids);


  }

  res.success = true;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "attach_object_server");
  
  MoveitInterface mai;

  ros::spin();

  return 0;
}