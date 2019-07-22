#include <moveit/move_group_interface/move_group_interface.h>
#include <chrono>
#include <thread>

#include "trajectory_utils.h"
#include "homing.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "cartesian_plan_tutorial");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  move_home();

  geometry_msgs::Pose home;
  home.orientation.x =  .707;
  home.orientation.y =     0;
  home.orientation.z = -.707;
  home.orientation.w =     0;
  home.position.x = -.06;
  home.position.y =  .319;
  home.position.z =  .484;

  geometry_msgs::Pose start;
  start.orientation.x = -.5;
  start.orientation.y =  .5;
  start.orientation.z =  .5;
  start.orientation.w =  .5;
  start.position.x = -.032;
  start.position.y =  .377;
  start.position.z =  .218;

  double scan_angle = 20.0 * M_PI/180;
  double f = .15;

  tf::Quaternion qi;
  tf::quaternionMsgToTF(start.orientation, qi);

  geometry_msgs::Pose start_tilt = start;
  tf::Quaternion q1(
      tf::Vector3(0, 1, 0),
      -scan_angle);
  tf::quaternionTFToMsg(qi*q1, start_tilt.orientation);
  start_tilt.position.y = start.position.y - f * tan(scan_angle);

  geometry_msgs::Pose end_tilt1 = start_tilt;
  end_tilt1.position.x += 0.3;

  geometry_msgs::Pose end_tilt2 = end_tilt1;
  tf::Quaternion q2(
      tf::Vector3(0, 1, 0),
     scan_angle);
  tf::quaternionTFToMsg(qi*q2, end_tilt2.orientation);
  end_tilt2.position.y = start.position.y + f * tan(scan_angle);

  geometry_msgs::Pose start_tilt2 = end_tilt2;
  start_tilt2.position.x = start.position.x;

  

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start);
  waypoints.push_back(end_tilt1); // end_tilt1 // start_tilt
  // printf("waypoints npoints: %ld\n", waypoints.size());

  std::vector<geometry_msgs::Pose> scan_waypoints;
  scan_waypoints.push_back(end_tilt1); // end_tilt1 // start_tilt
  scan_waypoints.push_back(start_tilt); // start_tilt // end_tilt1
 
  // printf("scan_waypoints npoints: %ld\n", waypoints.size());

  std::vector<geometry_msgs::Pose> turn_waypoints;
  turn_waypoints.push_back(start_tilt); // start_tilt // end_tilt1
  turn_waypoints.push_back(start_tilt2); // start_tilt2 // end_tilt2

  std::vector<geometry_msgs::Pose> scan2_waypoints;
  scan2_waypoints.push_back(start_tilt2); // start_tilt2 // end_tilt2
  scan2_waypoints.push_back(end_tilt2); // end_tilt2 // start_tilt2

  std::vector<geometry_msgs::Pose> turn2_waypoints;
  turn2_waypoints.push_back(end_tilt2); // end_tilt2 // start_tilt2
  turn2_waypoints.push_back(start); // start // start

  const double jump_threshold = 5;
  const double eef_step = 0.01;
  double fraction;
  moveit_msgs::RobotTrajectory trajectory;

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan4;

  fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("moveit_line", "waypoints (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
  rescale_trajectory(&trajectory, &my_plan.trajectory_, 0.20);
  move_group.execute(my_plan);

  printf("finished 1\n");
  move_group.setStartStateToCurrentState();
  fraction = move_group.computeCartesianPath(scan_waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("moveit_line", "scan_waypoints (%.2f%% acheived)", fraction * 100.0);
  rescale_trajectory(&trajectory, &my_plan1.trajectory_, 0.01);
  move_group.execute(my_plan1);

  printf("finished 2\n");
  move_group.setStartStateToCurrentState();
  fraction = move_group.computeCartesianPath(turn_waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("moveit_line", "turn_waypoints (%.2f%% acheived)", fraction * 100.0);
  rescale_trajectory(&trajectory, &my_plan2.trajectory_, 0.20);
  move_group.execute(my_plan2);

  move_group.setStartStateToCurrentState();
  fraction = move_group.computeCartesianPath(scan2_waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("moveit_line", "scan2_waypoints (%.2f%% acheived)", fraction * 100.0);
  rescale_trajectory(&trajectory, &my_plan3.trajectory_, 0.01);
  move_group.execute(my_plan3);

  move_group.setStartStateToCurrentState();
  fraction = move_group.computeCartesianPath(turn2_waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("moveit_line", "turn2_waypoints (%.2f%% acheived)", fraction * 100.0);
  rescale_trajectory(&trajectory, &my_plan4.trajectory_, 0.40);
  move_group.execute(my_plan4);

  return 0;
}
