#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

//#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "moveit_test");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.

  // static const std::string PLANNING_GROUP = "manipulator";
  static const std::string PLANNING_GROUP = "panda_arm";
  
  // The :move_group_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  // moveit::planning_interface::MoveGroup group(PLANNING_GROUP);

  // group->setEndEffectorLink("");
  
  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("Test", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("Test", "End effector link: %s", move_group.getEndEffectorLink().c_str());


  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();

  geometry_msgs::Pose target_pose1 = current_pose.pose;
  target_pose1.orientation.w = current_pose.pose.orientation.w;
  target_pose1.position.x = current_pose.pose.position.x + 0.1;
  target_pose1.position.y = current_pose.pose.position.y;
  target_pose1.position.z = current_pose.pose.position.z;
  move_group.setPoseTarget(target_pose1);

  ROS_INFO("Current pose, position x: %f", current_pose.pose.position.x);
  ROS_INFO("Current pose, position y: %f", current_pose.pose.position.y);
  ROS_INFO("Current pose, position z: %f", current_pose.pose.position.z);   

  ROS_INFO("Target pose, position x: %f", target_pose1.position.x);
  ROS_INFO("Target pose, position y: %f", target_pose1.position.y);
  ROS_INFO("Target pose, position z: %f", target_pose1.position.z);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("Test", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  ros::shutdown();
  return 0;
}
