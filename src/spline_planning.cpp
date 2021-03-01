#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <stomp_moveit/stomp_planner.h>
#include <stomp_moveit/stomp_planner_manager.h>
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
    move_group.setPlanningTime(5);
    // Load plugin planner
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


    // Waypoints definition
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose tpose_1;
    tpose_1.position.x = 0.03;
    tpose_1.position.y = 0.03;
    tpose_1.position.z = -0.05;
    tpose_1.orientation.w = 0.8;
//    start_state.setFromIK(joint_model_group, tpose_1);
    move_group.setStartState(start_state);
    waypoints.push_back(tpose_1);

    geometry_msgs::Pose tpose_2;
    tpose_2.position.x = 0.06;
    tpose_2.position.y = 0.06;
    tpose_2.position.z = -0.1;
    tpose_2.orientation.w = 1.0;
    waypoints.push_back(tpose_2);

    geometry_msgs::Pose tpose_3;
    tpose_3.position.x = 0.15;
    tpose_3.position.y = 0.15;
    tpose_3.position.z = -0.08;
    tpose_3.orientation.w = 1;
    waypoints.push_back(tpose_3);

    move_group.setMaxVelocityScalingFactor(0.08);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    geometry_msgs::PoseStamped wp1;
    wp1.header.frame_id = "world";
    wp1.pose = tpose_1;

    geometry_msgs::PoseStamped wp2;
    wp1.header.frame_id = "world";
    wp1.pose = tpose_2;

    geometry_msgs::PoseStamped wp3;
    wp1.header.frame_id = "world";
    wp1.pose = tpose_3;

    geometry_msgs::PoseStamped pose_end;
    pose_end.header.frame_id = "world";
    pose_end.pose = tpose_3;

    moveit_msgs::Constraints pose_goal_end =
            kinematic_constraints::constructGoalConstraints("psm_tool_tip_link", pose_end, tolerance_pose, tolerance_angle);

    moveit_msgs::Constraints wp1_cons =
            kinematic_constraints::constructGoalConstraints("psm_tool_tip_link", wp2, tolerance_pose, tolerance_angle);

    moveit_msgs::Constraints wp2_cons =
            kinematic_constraints::constructGoalConstraints("psm_tool_tip_link", wp2, tolerance_pose, tolerance_angle);

//    req.goal_constraints.push_back(wp_goal);
    req.goal_constraints.push_back(pose_goal_end);
    req.allowed_planning_time = 10.;
    req.trajectory_constraints = stomp_moveit::StompPlanner::encodeSeedTrajectory(trajectory.joint_trajectory);
//    req.trajectory_constraints.constraints.push_back(wp_cons);
    moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);
//    req.start_state = start_state;
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
    visual_tools.publishAxisLabeled(tpose_1, "goal_1");
    visual_tools.publishAxisLabeled(tpose_2, "goal_2");
    visual_tools.publishAxisLabeled(tpose_3, "goal_3");

    visual_tools.trigger();


    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

}

