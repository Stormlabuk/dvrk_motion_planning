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
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);
    float max_vel_scaling_factor = .04;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
}