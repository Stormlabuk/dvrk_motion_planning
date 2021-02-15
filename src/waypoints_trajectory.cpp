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

    const std::string node_name = "motion_planning_tutorial";
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

    geometry_msgs::Pose tpose_1;
    tpose_1.orientation.w = 1.0;
    tpose_1.position.x = 0.05;
    tpose_1.position.y = 0.05;
    tpose_1.position.z = -0.15;
    move_group.setPoseTarget(tpose_1);

//    geometry_msgs::Pose tpose_2;
//    tpose_2.position.x = 0.1;
//    tpose_2.position.y = 0.1;
//    tpose_2.position.z = -0.1;
//    tpose_2.orientation.w = 1.0;
//    move_group.push_back(tpose_2);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(tpose_1, "pose1");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


//    std::vector<geometry_msgs::Pose> waypoints;
//
//    // First point
//    geometry_msgs::Pose pose_1;
//    pose_1.position.x = 0.05;
//    pose_1.position.y = 0.05;
//    pose_1.position.z = -0.15;
//    pose_1.orientation.w = 1.0;
//    waypoints.push_back(pose_1);
//
//    geometry_msgs::Pose pose_2;
//
//    pose_2.position.x = 0.1;
//    pose_2.position.y = 0.1;
//    pose_2.position.z = -0.1;
//    pose_2.orientation.w = 1.0;
//    waypoints.push_back(pose_2);
//
//    moveit_msgs::RobotTrajectory trajectory;
//    double fraction = move_group.computeCartesianPath(waypoints,
//                                                 0.01,  // eef_step
//                                                 0.0,   // jump_threshold
//                                                 trajectory);


}