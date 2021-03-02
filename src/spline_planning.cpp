#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <stomp_moveit/stomp_planner.h>
#include <stomp_moveit/stomp_planner_manager.h>
#include <dvrk_moveit_class.h>
#include <string.h>

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

    const std::string node_name = "spline_planning";
    ros::init(argc, argv, node_name);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    const std::string PLANNING_GROUP = "psm_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");
//    move_group.setPlanningTime(5);


    planning_interface::PlannerManagerPtr planner_instance = MoveItDVRKPlanning::loadPlannerPlugin(node_handle,robot_model);


    // ################
    // ### VISUALIS ###
    // ################
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();  // clear all old markers
    visual_tools.trigger();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();

    // ################
    // ### PLANNING ###
    // ################
    visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
    visual_tools.trigger();
    planning_interface::MotionPlanRequest req;
    req.group_name = PLANNING_GROUP;
    planning_interface::MotionPlanResponse res;
    robot_state::RobotState start_state(*move_group.getCurrentState());


    std::vector<geometry_msgs::Pose> waypoints;
    waypoints = MoveItDVRKPlanning::getWaypointsVector('L');

    start_state.setFromIK(joint_model_group, waypoints.at(0));
    move_group.setStartState(start_state);

    move_group.setMaxVelocityScalingFactor(0.04);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    geometry_msgs::PoseStamped wp1;
    wp1.header.frame_id = "world";
    wp1.pose = waypoints.at(0);

    geometry_msgs::PoseStamped wp2;
    wp2.header.frame_id = "world";
    wp2.pose = waypoints.at(1);

    geometry_msgs::PoseStamped wp3;
    wp3.header.frame_id = "world";
    wp3.pose = waypoints.at(2);

    geometry_msgs::PoseStamped pose_end;
    pose_end.header.frame_id = "world";
    pose_end.pose = waypoints.at(2);

    moveit_msgs::Constraints pose_goal_end =
            kinematic_constraints::constructGoalConstraints("psm_tool_tip_link", pose_end, tolerance_pose, tolerance_angle);

    moveit_msgs::Constraints wp1_cons =
            kinematic_constraints::constructGoalConstraints("psm_tool_tip_link", wp2, tolerance_pose, tolerance_angle);

    moveit_msgs::Constraints wp2_cons =
            kinematic_constraints::constructGoalConstraints("psm_tool_tip_link", wp2, tolerance_pose, tolerance_angle);

    req.goal_constraints.push_back(pose_goal_end);
    req.allowed_planning_time = 10.;
    req.trajectory_constraints = stomp_moveit::StompPlanner::encodeSeedTrajectory(trajectory.joint_trajectory);
    moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);
    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);

    context->setMotionPlanRequest(req);

    context->solve(res);

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

