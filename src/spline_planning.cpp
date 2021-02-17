#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//
//
//#include <boost/scoped_ptr.hpp>

int main(int argc, char** argv) {

    const std::string node_name = "waypoints_trajectory";
    ros::init(argc, argv, node_name);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    moveit::planning_interface::MoveGroupInterface move_group("psm_arm"); // definition of current move_group
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    planning_pipeline::PlanningPipelinePtr planning_pipeline(
            new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));
    const robot_state::JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup("psm_arm");// pointer for improved performance
    robot_state::RobotState start_state(*move_group.getCurrentState());

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();

    visual_tools.loadRemoteControl();

    visual_tools.trigger();
    ros::Duration(10).sleep();

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    req.group_name = "psm_arm";

    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    // Waypoints definition
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose tpose_1;
    tpose_1.position.x = 0.05;
    tpose_1.position.y = 0.05;
    tpose_1.position.z = -0.15;
    tpose_1.orientation.w = 0.08;

    start_state.setFromIK(joint_model_group, tpose_1);
    move_group.setStartState(start_state);
    waypoints.push_back(tpose_1);

    geometry_msgs::Pose tpose_2;
    tpose_2.position.x = 0.1;
    tpose_2.position.y = 0.1;
    tpose_2.position.z = -0.1;
    tpose_2.orientation.w = 1.0;
    waypoints.push_back(tpose_2);

    geometry_msgs::Pose tpose_3;
    tpose_3.position.x = 0.0;
    tpose_3.position.y = 0.0;
    tpose_3.position.z = 0.0;
    tpose_3.orientation.w = 0.5;
    waypoints.push_back(tpose_3);

    move_group.setMaxVelocityScalingFactor(0.1);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

#include <movmoveit_msgs::MotionPlanResponse response;eit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>moveit_msgs::MotionPlanResponse response;moveit_msgs::MotionPlanResponse response;

    robot_state = planning_scene->getCurrentStateNonConst();
    planning_scene->setCurrentState(response.trajectory_start);
    robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
}