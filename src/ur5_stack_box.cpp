/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/Bool.h>

//#define DEBUG
#define DELAY 0.4

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  ros::Publisher magnet_on_pub = node_handle.advertise<std_msgs::Bool>("magnet_on", 1000);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "manipulator";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setPlanningTime(20);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";

  // The id of the object is used to identify it.
  collision_object.id = "table";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 2.0;
  primitive.dimensions[1] = 0.5;
  primitive.dimensions[2] = 0.1;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.0;
  box_pose.position.y = 0.10;
  box_pose.position.z = -0.06;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  moveit_msgs::CollisionObject monitor;
  monitor.header.frame_id = "base_link";

  // The id of the object is used to identify it.
  monitor.id = "monitor";

  // Define a box to add to the world.
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.55;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 2;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose monitor_pose;
  monitor_pose.orientation.w = 1.0;
  monitor_pose.position.x = -0.2;
  monitor_pose.position.y = 0.4;
  monitor_pose.position.z = 1;

  monitor.primitives.push_back(primitive);
  monitor.primitive_poses.push_back(monitor_pose);
  monitor.operation = monitor.ADD;

  collision_objects.push_back(monitor);
  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  moveit_msgs::CollisionObject wall;
  wall.header.frame_id = "base_link";

  // The id of the object is used to identify it.
  wall.id = "wall";

  // Define a box to add to the world.
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 2;
  primitive.dimensions[2] = 2;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose wall_pose;
  wall_pose.orientation.w = 1.0;
  wall_pose.position.x = -0.715;
  wall_pose.position.y = 0;
  wall_pose.position.z = 0;

  wall.primitives.push_back(primitive);
  wall.primitive_poses.push_back(wall_pose);
  wall.operation = wall.ADD;

  collision_objects.push_back(wall);
  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  std_msgs::Bool magnet_on_msg;

  for(int for_count=0; for_count<2; for_count++) {
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();   
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    joint_group_positions[0] = 0;  // radians
    joint_group_positions[1] = -1.5;  // radians
    joint_group_positions[2] = 1.5;  // radians
    joint_group_positions[3] = -1.5;  // radians
    joint_group_positions[4] = -1.5;  // radians
    move_group.setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

#ifdef DEBUG
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
#else
    ros::Duration(DELAY).sleep();
#endif


    move_group.move(); //move to first position

    magnet_on_msg.data = true;
    magnet_on_pub.publish(magnet_on_msg);

    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.
    tf2::Quaternion q;
    q.setRPY(1.57, 1.57, 0);
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.x = q[0];
    target_pose1.orientation.y = q[1];
    target_pose1.orientation.z = q[2];
    target_pose1.orientation.w = q[3];
    target_pose1.position.x = 0.30;
    target_pose1.position.y = 0.25;
    if(for_count == 0)
      target_pose1.position.z = 0.286;
    else if(for_count == 1)
      target_pose1.position.z = 0.244;
    else
      target_pose1.position.z = 0.244;
    //target_pose1.position.x = 0.28;
    //target_pose1.position.y = -0.2;
    //target_pose1.position.z = 0.5;
    move_group.setPoseTarget(target_pose1);

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in RViz.
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

#ifdef DEBUG
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
#else
    ros::Duration(DELAY).sleep();
#endif

    move_group.move(); //move to first position

    q.setRPY(1.57, 1.57, 0);
    target_pose1.orientation.x = q[0];
    target_pose1.orientation.y = q[1];
    target_pose1.orientation.z = q[2];
    target_pose1.orientation.w = q[3];
    target_pose1.position.x = 0.30;
    target_pose1.position.y = 0.25;
    target_pose1.position.z = 0.33;
    //target_pose1.position.x = 0.28;
    //target_pose1.position.y = -0.2;
    //target_pose1.position.z = 0.5;
    move_group.setPoseTarget(target_pose1);

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in RViz.
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

#ifdef DEBUG
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
#else
    ros::Duration(DELAY).sleep();
#endif

    move_group.move(); //move to first position

    current_state = move_group.getCurrentState();   
    // Next get the current set of joint values for the group.
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    joint_group_positions[0] = -2.7;  // radians
    move_group.setJointValueTarget(joint_group_positions);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

#ifdef DEBUG
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
#else
    ros::Duration(DELAY).sleep();
#endif

    move_group.move(); //move to first position

    ros::Duration(0.8).sleep();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose3);

    if(for_count == 0)
      target_pose3.position.z -= 0.30;
    else if(for_count == 1)
      target_pose3.position.z -= 0.15;
    else
      target_pose3.position.z -= 0.15;

    waypoints.push_back(target_pose3);  // down

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.005;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;
    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

#ifdef DEBUG
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
#else
    //ros::Duration(DELAY).sleep();
#endif

    move_group.execute(cartesian_plan); //move to first position

    ros::Duration(0.8).sleep();
    magnet_on_msg.data = false;
    magnet_on_pub.publish(magnet_on_msg);
    ros::Duration(0.8).sleep();
    magnet_on_msg.data = true;
    magnet_on_pub.publish(magnet_on_msg);

  }

  ros::shutdown();
  return 0;
}
