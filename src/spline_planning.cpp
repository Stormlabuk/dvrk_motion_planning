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

    MoveItDVRKPlanning mid;

    // ### LOAD MOVE GROUP, ROBOT MODEL AND ROBOT STATE ###
    moveit::planning_interface::MoveGroupInterface move_group(mid.move_group_name);
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(mid.move_group_name);

    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

    // ### LOAD PLANNER PLUGIN ###
    planning_interface::PlannerManagerPtr planner_instance = MoveItDVRKPlanning::loadPlannerPlugin(node_handle,robot_model);

    // ################
    // ### VISUALIS ###
    // ################
//    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();  // clear all old markers
    visual_tools.trigger();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();
    visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
    visual_tools.trigger();

    // ################
    // ### PLANNING ###
    // ################
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name = mid.move_group_name;
    robot_state::RobotState start_state(*move_group.getCurrentState());

    mid.waypoints = MoveItDVRKPlanning::getWaypointsVector('L');

    start_state.setFromIK(joint_model_group, mid.waypoints.at(0));
    move_group.setStartState(start_state);
    move_group.setMaxVelocityScalingFactor(mid.max_vel_scaling_factor);

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(mid.waypoints, mid.eef_step, mid.jump_threshold, trajectory);

    geometry_msgs::PoseStamped pose_end;
    pose_end.header.frame_id = "world";
    pose_end.pose = mid.waypoints.at(2);

    moveit_msgs::Constraints pose_goal_end =
            kinematic_constraints::constructGoalConstraints("psm_tool_tip_link", pose_end, mid.tolerance_pose, mid.tolerance_angle);

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
    visual_tools.publishAxisLabeled(mid.waypoints.at(0), "goal_1");
    visual_tools.publishAxisLabeled(mid.waypoints.at(1), "goal_2");
    visual_tools.publishAxisLabeled(mid.waypoints.at(2), "goal_3");

    visual_tools.trigger();


    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

}

