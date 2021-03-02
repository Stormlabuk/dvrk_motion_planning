//
// Created by aleks on 01/03/2021.
//


#ifndef DVRK_MOVEIT_DVRK_MOVEIT_CLASS_H
#define DVRK_MOVEIT_DVRK_MOVEIT_CLASS_H

#include <stdio.h>
#include <geometry_msgs/Pose.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>

class MoveItDVRKPlanning {


public:
    MoveItDVRKPlanning();
    static std::vector<geometry_msgs::Pose> getWaypointsVector(char traj_ID);
    static planning_interface::PlannerManagerPtr loadPlannerPlugin(ros::NodeHandle node_handle, robot_model::RobotModelPtr robot_model);
};


#endif //DVRK_MOVEIT_DVRK_MOVEIT_CLASS_H
