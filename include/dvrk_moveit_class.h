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

class MoveItDVRKPlanning {


public:
    std::vector<double> tolerance_pose;
    std::vector<double> tolerance_angle;
    float max_vel_scaling_factor;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    std::string move_group_name = "psm_arm";
    std::vector<geometry_msgs::Pose> waypoints;
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;


    MoveItDVRKPlanning();
    static std::vector<geometry_msgs::Pose> getWaypointsVector(char traj_ID);
    static planning_interface::PlannerManagerPtr loadPlannerPlugin(ros::NodeHandle node_handle, robot_model::RobotModelPtr robot_model);
    static void setupRVizVisualisation(moveit_visual_tools::MoveItVisualTools visual_tools,  planning_scene::PlanningScenePtr planning_scene);
    moveit_msgs::Constraints computeGoalConstraint(geometry_msgs::Pose goal_pose);
    void compileMotionPlanRequest(moveit_msgs::Constraints goal_constraint, moveit_msgs::RobotTrajectory trajectory, robot_state::RobotState start_state);
};


#endif //DVRK_MOVEIT_DVRK_MOVEIT_CLASS_H
