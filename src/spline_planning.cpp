//#include <pluginlib/class_loader.h>
#include <ros/ros.h>
//#include <stomp_moveit/stomp_planner.h>
//#include <stomp_moveit/stomp_planner_manager.h>
#include <dvrk_moveit_class.h>
#include <string.h>

// MoveIt!
//#include <moveit/robot_model_loader/robot_model_loader.h>
//#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
//#include <moveit_msgs/PlanningScene.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>
//#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv) {

    const std::string node_name = "spline_planning";
    ros::init(argc, argv, node_name);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    MoveItDVRKPlanning mid;

    // ### LOAD PLANNER PLUGIN ###
    planning_interface::PlannerManagerPtr planner_instance = mid.loadPlannerPlugin(node_handle);

    // ### SETUP PLANNING SCENE ###
    mid.setupPlanningScene();

    // ### DEFINE WAYPOINTS ###
    mid.waypoints = mid.getWaypointsVector('L');
    ROS_INFO("### Waypoints pose validity check:");
    std::vector<bool> wp_validity;
    for (int i = 0; i<3; i++){
        if(mid.move_group.setJointValueTarget(mid.waypoints.at(i),"psm_tool_tip_link")){
            ROS_INFO("### Waypoints #%d: VALID", i);
        } else{ ROS_ERROR("### Waypoints #%d: INVALID",i);}
    }



    // EVALUATE CARTESIAN PATH TO SMOOTH WITH STOMP
    mid.start_state.setFromIK(mid.joint_model_group, mid.home_pose); // set start state as first point of waypoints
    mid.move_group.setStartState(mid.start_state);
    mid.move_group.setMaxVelocityScalingFactor(mid.max_vel_scaling_factor);

    // Check validity of waypoints
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = mid.move_group.computeCartesianPath(mid.waypoints, mid.eef_step, mid.jump_threshold, trajectory);

    // ### DEFINE GOAL POSE AND COMPILE MOTION PLAN REQUEST ###
    moveit_msgs::Constraints pose_goal_end = mid.computeGoalConstraint(mid.waypoints.at(2));
    mid.compileMotionPlanRequest(pose_goal_end, trajectory);

    // SOLVE REQUEST
    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(mid.planning_scene, mid.req, mid.res.error_code_);
    context->setMotionPlanRequest(mid.req);
    context->solve(mid.res);

    // ### SHOW RESULT TRAJECTORY ###
    mid.displayResultTrajectory(node_handle);

}

