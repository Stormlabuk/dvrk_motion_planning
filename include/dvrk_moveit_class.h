//
// Created by aleks on 01/03/2021.
//


#ifndef DVRK_MOVEIT_DVRK_MOVEIT_CLASS_H
#define DVRK_MOVEIT_DVRK_MOVEIT_CLASS_H

#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <memory>

class MoveItDVRKPlanning {


public:
    std::string move_group_name = "psm_arm";
    std::string arm_name;
    std::vector<geometry_msgs::Pose> waypoints;
    std::vector<double> tolerance_pose;
    std::vector<double> tolerance_angle;
    int dvrk_version;
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    float max_vel_scaling_factor;
    const double jump_threshold = 0.0;
    const double eef_step = 0.001;
    geometry_msgs::Pose home_pose;


    ros::Publisher cartesian_pub;


    moveit::planning_interface::MoveGroupInterface move_group = moveit::planning_interface::MoveGroupInterface(move_group_name);
    robot_model_loader::RobotModelLoader robot_model_loader = robot_model_loader::RobotModelLoader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr robot_state = std::make_shared<robot_state::RobotState>(robot_model);
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(move_group_name);
    robot_state::RobotState start_state = robot_state::RobotState(*move_group.getCurrentState()); // initial state of the robot
    planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    moveit_visual_tools::MoveItVisualTools visual_tools = moveit_visual_tools::MoveItVisualTools("world");


    MoveItDVRKPlanning(std::string arm, int version = 1);
    std::vector<geometry_msgs::Pose> getWaypointsVector(char traj_ID);
    std::vector<geometry_msgs::Pose> getRandomWaypointsVector(int n, std::string eef_name = "psm_tool_tip_link");
    planning_interface::PlannerManagerPtr loadPlannerPlugin(ros::NodeHandle node_handle);
    moveit_msgs::Constraints computeGoalConstraint(geometry_msgs::Pose goal_pose);
    std::vector<geometry_msgs::Pose> convertJointTrajectoryToCartesian ();
    std::vector<sensor_msgs::JointState> convertJointTrajectoryToJointState ();
    void setupDVRKTrajectoryPublisher(ros::NodeHandle node_handle);
    void setupPlanningScene();
    void compileMotionPlanRequest(moveit_msgs::Constraints goal_constraint, moveit_msgs::RobotTrajectory trajectory);
    void displayResultTrajectory(ros::NodeHandle node_handle);
    void displayWaypoints();
    void checkWaypointsValidity(std::vector<geometry_msgs::Pose> wp_vector);
    void checkPoseValidity(geometry_msgs::Pose);

};


#endif //DVRK_MOVEIT_DVRK_MOVEIT_CLASS_H
