#include <ros/ros.h>
#include <dvrk_moveit_class.h>
#include <string.h>

// MoveIt!
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>


int main(int argc, char** argv) {

    ros::init(argc, argv, "spline_planning");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    MoveItDVRKPlanning mid("PSM1", 1);

    // ### SETUP PUBLISHERS FOR
    mid.setupDVRKCartesianTrajectoryPublisher(node_handle);
    mid.setupDVRKJointTrajectoryPublisher(node_handle);

    // ### LOAD PLANNER PLUGIN ###
    planning_interface::PlannerManagerPtr planner_instance = mid.loadPlannerPlugin(node_handle);

    // ### SETUP PLANNING SCENE ###
    mid.setupPlanningScene();

    // ### DEFINE AND VALIDATE WAYPOINTS ###
    mid.waypoints = mid.getWaypointsVector('B');
    mid.checkPoseValidity(mid.home_pose);
    mid.checkWaypointsValidity(mid.waypoints);

    // SETUP INITIAL STATE
    mid.start_state.setFromIK(mid.joint_model_group, mid.home_pose); // set start state as home_pose
    mid.move_group.setStartState(mid.start_state);
    mid.move_group.setMaxVelocityScalingFactor(mid.max_vel_scaling_factor);

    // ### EVALUATE CARTESIAN PATH TO SMOOTH WITH STOMP ###
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = mid.move_group.computeCartesianPath(mid.waypoints, mid.eef_step, mid.jump_threshold, trajectory) * 100;

    // ### DEFINE GOAL POSE AND COMPILE MOTION PLAN REQUEST ###
    moveit_msgs::Constraints pose_goal_end = mid.computeGoalConstraint(mid.waypoints.at(2));
    mid.compileMotionPlanRequest(pose_goal_end, trajectory);

    // SOLVE REQUEST
    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(mid.planning_scene, mid.req, mid.res.error_code_);
    context->setMotionPlanRequest(mid.req);
    context->solve(mid.res);

    std::vector<sensor_msgs::JointState> joint_trajectory = mid.convertJointTrajectoryToJointState();
    std::vector<geometry_msgs::Pose> pose_trajectory = mid.convertJointTrajectoryToCartesian();

    // ### SHOW RESULT TRAJECTORY ###
    mid.displayResultTrajectory(node_handle);

    // ### STOP SPINNER AND DEFINE NEW ROS SPINNER ###
    spinner.stop();
    ros::Rate r(50);

//    fraction = fraction * 100;

    if(fraction > 95) {
        ROS_INFO("!!! Planning successful: %.03f percent of the trajectory is followed.", fraction);
        ROS_INFO("Publishing trajectory of %d points", (int) pose_trajectory.size());

        while (ros::ok()) {
            for (int i = 0; i < pose_trajectory.size(); i++) {

                mid.cartesian_pub.publish(pose_trajectory.at(i));
                mid.joint_pub.publish(joint_trajectory.at(i));

                ros::spinOnce();
                r.sleep();
            }
            break;
        }
        ROS_INFO("Trajectory published and executed. Shutting down node...");
    }

    return 0;
}

