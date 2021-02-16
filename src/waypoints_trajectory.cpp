#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
//#include <moveit/robot_model_loader/robot_model_loader.h>
//#include <moveit/planning_interface/planning_interface.h>
//#include <moveit/planning_scene/planning_scene.h>
//#include <moveit/kinematic_constraints/utils.h>
//#include <moveit_msgs/DisplayTrajectory.h>
//#include <moveit_msgs/PlanningScene.h>

//
//
//#include <boost/scoped_ptr.hpp>

int main(int argc, char** argv) {

    const std::string node_name = "waypoints_trajectory";
    ros::init(argc, argv, node_name);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    // Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group
    const std::string PLANNING_GROUP = "psm_arm"; // move group name
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP); // definition of current move_group
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // collisions and external objects
    const robot_state::JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);// pointer for improved performance

    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;

    // RViz Visual Tools
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl(); // load RVizVisualToolsGui
    visual_tools.trigger();

    robot_state::RobotState start_state(*move_group.getCurrentState());

    // Waypoints definition
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose tpose_1;
    tpose_1.orientation.w = 1.0;
    tpose_1.position.x = 0.05;
    tpose_1.position.y = 0.05;
    tpose_1.position.z = -0.15;
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
    tpose_3.orientation.w = 1.0;
    waypoints.push_back(tpose_3);

    move_group.setMaxVelocityScalingFactor(0.1);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    visual_tools.deleteAllMarkers();
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

}