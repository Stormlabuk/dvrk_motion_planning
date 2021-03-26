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

    MoveItDVRKPlanning mid("PSM1", 2);

    // ### SETUP PUBLISHERS FOR
    mid.setupDVRKTrajectoryPublisher(node_handle);

    // ### LOAD PLANNER PLUGIN ###
    planning_interface::PlannerManagerPtr planner_instance = mid.loadPlannerPlugin(node_handle);

    // ### SETUP PLANNING SCENE ###
    mid.setupPlanningScene();

    // ### DEFINE AND VALIDATE WAYPOINTS ###
    mid.waypoints = mid.getWaypointsVector('W');
    mid.checkPoseValidity(mid.home_pose);
    mid.checkWaypointsValidity(mid.waypoints);

    // SETUP INITIAL STATE
    mid.start_state.setFromIK(mid.joint_model_group, mid.home_pose); // set start state as home_pose
    mid.move_group.setStartState(mid.start_state);
    mid.move_group.setMaxVelocityScalingFactor(mid.max_vel_scaling_factor);

    // ### EVALUATE CARTESIAN PATH TO SMOOTH WITH STOMP ###
    moveit_msgs::RobotTrajectory trajectory;
    mid.move_group.computeCartesianPath(mid.waypoints, mid.eef_step, mid.jump_threshold, trajectory);

    // ### DEFINE GOAL POSE AND COMPILE MOTION PLAN REQUEST ###
    moveit_msgs::Constraints pose_goal_end = mid.computeGoalConstraint(mid.waypoints.at(mid.waypoints.size()-1));
    mid.compileMotionPlanRequest(pose_goal_end, trajectory);

    // ############################### COLLISION AVOIDANCE #######################
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = mid.move_group.getPlanningFrame();
    collision_object.id = "sph1";


    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.SPHERE;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.02;
    primitive.dimensions[1] = 0.02;
    primitive.dimensions[2] = 0.02;

    geometry_msgs::Pose sph_pose;
    sph_pose.position.x = 0;
    sph_pose.position.y = 0.03;
    sph_pose.position.z = -0.05;
    sph_pose.orientation.w = 1.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(sph_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> cos;
    cos.push_back(collision_object);

    mid.planning_scene_interface.addCollisionObjects(cos);
    // ######################################################################################

    // SOLVE REQUEST
    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(mid.planning_scene, mid.req, mid.res.error_code_);
    context->setMotionPlanRequest(mid.req);
    context->solve(mid.res);

//    std::vector<sensor_msgs::JointState> joint_trajectory = mid.convertJointTrajectoryToJointState();
    std::vector<geometry_msgs::Pose> pose_trajectory = mid.convertJointTrajectoryToCartesian();
//
    // ### SHOW RESULT TRAJECTORY ###
    mid.displayResultTrajectory(node_handle);


}

