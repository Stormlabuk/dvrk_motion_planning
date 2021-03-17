#include <ros/ros.h>
#include <dvrk_moveit_class.h>
#include <string.h>
#include <moveit_msgs/GetPositionFK.h>

// MoveIt!
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>


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
    mid.waypoints = mid.getWaypointsVector('B');
    mid.checkPoseValidity(mid.home_pose);
    mid.checkWaypointsValidity(mid.waypoints);

    // EVALUATE CARTESIAN PATH TO SMOOTH WITH STOMP
    mid.start_state.setFromIK(mid.joint_model_group, mid.home_pose); // set start state as home_pose
    mid.move_group.setStartState(mid.start_state);
    mid.move_group.setMaxVelocityScalingFactor(mid.max_vel_scaling_factor);

    // Check validity of waypoints
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = mid.move_group.computeCartesianPath(mid.waypoints, mid.eef_step, mid.jump_threshold, trajectory);

    // ### DEFINE GOAL POSE AND COMPILE MOTION PLAN REQUEST ###
    moveit_msgs::Constraints pose_goal_end = mid.computeGoalConstraint(mid.waypoints.at(2));
    mid.compileMotionPlanRequest(pose_goal_end, trajectory);

    std::vector<geometry_msgs::Pose> pose_trajectory;

    for (int i = 0; i<trajectory.joint_trajectory.points.size(); i++) {
        std::vector<double> joint_values;

        for (int k = 0; k<trajectory.joint_trajectory.joint_names.size(); k++) {
            joint_values.push_back(trajectory.joint_trajectory.points[i].positions[k]);}

        //initialize joint values
        mid.robot_state->setJointGroupPositions(mid.joint_model_group->getName(), joint_values);
        const Eigen::Affine3d &link_pose = mid.robot_state->getGlobalLinkTransform("psm_tool_tip_link");
        Eigen::Vector3d cartesian_position = link_pose.translation();
        Eigen::Matrix3d link_orientation = link_pose.rotation();
        Eigen::Quaterniond rot_quat(link_orientation);

        // populate Pose Message
        geometry_msgs::Pose tp;
        tp.position.x = cartesian_position.x();
        tp.position.y = cartesian_position.y();
        tp.position.z = cartesian_position.z();
        tp.orientation.w = rot_quat.w();
        tp.orientation.x = rot_quat.x();
        tp.orientation.y = rot_quat.y();
        tp.orientation.z = rot_quat.z();

        std::cout << "x: " << tp.position.x << std::endl;
        std::cout << "y: " << tp.position.y << std::endl;
        std::cout << "z: " << tp.position.z << std::endl;

        std::cout << "w: " << tp.orientation.w << std::endl;
        std::cout << "x: " << tp.orientation.x << std::endl;
        std::cout << "y: " << tp.orientation.y << std::endl;
        std::cout << "z: " << tp.orientation.z << std::endl;

        pose_trajectory.push_back(tp);

    }
    std::cout << "JOINT TRAJECTORY HAS POINT: " << trajectory.joint_trajectory.points.size() << std::endl;
    std::cout << "CARTESIAN TRAJECTORY HAS POINT " << pose_trajectory.size() << std::endl;

    // SOLVE REQUEST
    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(mid.planning_scene, mid.req, mid.res.error_code_);
    context->setMotionPlanRequest(mid.req);
    context->solve(mid.res);

    // ### SHOW RESULT TRAJECTORY ###
    mid.displayResultTrajectory(node_handle);

}

