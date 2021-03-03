//
// Created by aleks on 01/03/2021.
//
#include <string.h>
#include <geometry_msgs/Pose.h>
#include "dvrk_moveit_class.h"
#include <moveit/planning_interface/planning_interface.h>
#include <pluginlib/class_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <boost/scoped_ptr.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <stomp_moveit/stomp_planner.h>
#include <moveit/kinematic_constraints/utils.h>

std::vector<geometry_msgs::Pose> MoveItDVRKPlanning::getWaypointsVector(char traj_ID) {
    //waypooints vector
    std::vector<geometry_msgs::Pose> waypoints;

    // points -> waypoints
    geometry_msgs::Pose tpose_1;
    geometry_msgs::Pose tpose_2;
    geometry_msgs::Pose tpose_3;

    // waypoints for left trajectory (L)
    if(traj_ID == 'L'){
        tpose_1.position.x = 0.11;
        tpose_1.position.y = 0.07;
        tpose_1.position.z = -0.07;
        tpose_1.orientation.w = 0.4;

        tpose_2.position.x = 0.06;
        tpose_2.position.y = 0.05;
        tpose_2.position.z = -0.05;
        tpose_2.orientation.w = 1.0;

        tpose_3.position.x = 0.02;
        tpose_3.position.y = 0.09;
        tpose_3.position.z = -0.09;
        tpose_3.orientation.w = 1;
    }

    if (traj_ID == 'R'){

        tpose_1.position.x = -0.08;
        tpose_1.position.y = 0.02;
        tpose_1.position.z = -0.05;
        tpose_1.orientation.w = 0.4;

        tpose_2.position.x = 0.0;
        tpose_2.position.y = 0.05;
        tpose_2.position.z = -0.05;
        tpose_2.orientation.w = 1.0;

        tpose_3.position.x = 0.06;
        tpose_3.position.y = 0.02;
        tpose_3.position.z = -0.04;
        tpose_3.orientation.w = 1;
    }

    if (traj_ID == 'B'){

        tpose_1.position.x = 0.02;
        tpose_1.position.y = 0.03;
        tpose_1.position.z = -0.03;
        tpose_1.orientation.w = 0.4;

        tpose_2.position.x = -0.02;
        tpose_2.position.y = 0.05;
        tpose_2.position.z = -0.05;
        tpose_2.orientation.w = 1.0;

        tpose_3.position.x = 0.02;
        tpose_3.position.y = 0.08;
        tpose_3.position.z = -0.09;
        tpose_3.orientation.w = 1;
    }


    waypoints.push_back(tpose_1);
    waypoints.push_back(tpose_2);
    waypoints.push_back(tpose_3);

    return waypoints;
}

planning_interface::PlannerManagerPtr MoveItDVRKPlanning::loadPlannerPlugin(ros::NodeHandle node_handle, robot_model::RobotModelPtr robot_model) {
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    // Handle plugin boot
    if (!node_handle.getParam("/move_group/planning_plugin", planner_plugin_name))
        ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
                "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        std::string pippo = node_handle.getNamespace();
        if (!planner_instance->initialize(robot_model, "move_group"))
            ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0; i < classes.size(); ++i)
            ss << classes[i] << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                             << "Available plugins: " << ss.str());
    }

    return planner_instance;
}

MoveItDVRKPlanning::MoveItDVRKPlanning(){
    tolerance_pose = std::vector<double> (3,0.01);
    tolerance_angle = std::vector<double> (3,0.01);
    max_vel_scaling_factor = 0.04;
    planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");
}

void MoveItDVRKPlanning::setupPlanningScene() {
    visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();  // clear all old markers
    visual_tools.trigger();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();
    visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
    visual_tools.trigger();
}

moveit_msgs::Constraints MoveItDVRKPlanning::computeGoalConstraint(geometry_msgs::Pose goal_pose){

    geometry_msgs::PoseStamped pose_end;
    pose_end.header.frame_id = "world";
    pose_end.pose = goal_pose;

    moveit_msgs::Constraints goal_cons =
            kinematic_constraints::constructGoalConstraints("psm_tool_tip_link", pose_end, tolerance_pose, tolerance_angle);

    return goal_cons;

}

void MoveItDVRKPlanning::compileMotionPlanRequest(moveit_msgs::Constraints goal_constraint, moveit_msgs::RobotTrajectory trajectory, robot_state::RobotState start_state){
    req.group_name = move_group_name;
    req.goal_constraints.push_back(goal_constraint);
    req.allowed_planning_time = 10.;
    req.trajectory_constraints = stomp_moveit::StompPlanner::encodeSeedTrajectory(trajectory.joint_trajectory);
    moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);

}

void MoveItDVRKPlanning::displayResultTrajectory(ros::NodeHandle node_handle){
    ros::Publisher display_publisher =
            node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    display_publisher.publish(display_trajectory);

    robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    planning_scene->setCurrentState(*robot_state.get());

    visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
    visual_tools.publishAxisLabeled(waypoints.at(0), "goal_1");
    visual_tools.publishAxisLabeled(waypoints.at(1), "goal_2");
    visual_tools.publishAxisLabeled(waypoints.at(2), "goal_3");

    visual_tools.trigger();


    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
}