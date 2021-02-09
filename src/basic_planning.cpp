#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>


int main (int argc, char** argv){

    ros::init(argc, argv, "single_point_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;

    std::size_t count = 0;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    // check self-collision
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 1 (Home pose self-collision): Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    // check collision in random position
    robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    current_state.setToRandomPositions();
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 2 (Random self-collision): Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    // check collision between psm gripper and arm
    collision_request.group_name = "psm_gripper";
    current_state.setToRandomPositions();
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 3 (gripper-robot collision): Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    // check current state for collisions
    std::vector<double> joint_values = { 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0};
    const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup("psm_arm");
    current_state.setJointGroupPositions(joint_model_group, joint_values);
    ROS_INFO_STREAM("Test 4 (specific pose collision): Current state is "
                            << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));

    collision_request.contacts = true;
    collision_request.max_contacts = 1000;

    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 5: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
    {
        ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
    }

    // check allowed collisions
    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
    robot_state::RobotState copied_state = planning_scene.getCurrentState();

    collision_detection::CollisionResult::ContactMap::const_iterator it2;
    for (it2 = collision_result.contacts.begin(); it2 != collision_result.contacts.end(); ++it2)
    {
        acm.setEntry(it2->first.first, it2->first.second, true);
    }
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);
    ROS_INFO_STREAM("Test 6: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    collision_result.clear();
    planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
    ROS_INFO_STREAM("Test 7: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");


    std::string end_effector_name = joint_model_group->getLinkModelNames().back();

    geometry_msgs::PoseStamped desired_pose;
    desired_pose.pose.orientation.w = 1.0;
    desired_pose.pose.position.x = 0.3;
    desired_pose.pose.position.y = -0.185;
    desired_pose.pose.position.z = 0.5;
    desired_pose.header.frame_id = "world";
    moveit_msgs::Constraints goal_constraint =
            kinematic_constraints::constructGoalConstraints(end_effector_name, desired_pose);

    copied_state.setToRandomPositions();
    copied_state.update();
    bool constrained = planning_scene.isStateConstrained(copied_state, goal_constraint);
    ROS_INFO_STREAM("Test 8: Random state is " << (constrained ? "constrained" : "not constrained"));

    // There's a more efficient way of checking constraints (when you want
    // to check the same constraint over and over again, e.g. inside a
    // planner). We first construct a KinematicConstraintSet which
    // pre-processes the ROS Constraints messages and sets it up for quick
    // processing.
    kinematic_constraints::KinematicConstraintSet kinematic_constraint_set(kinematic_model);
    kinematic_constraint_set.add(goal_constraint, planning_scene.getTransforms());
    bool constrained_2 = planning_scene.isStateConstrained(copied_state, kinematic_constraint_set);
    ROS_INFO_STREAM("Test 9: Random state is " << (constrained_2 ? "constrained" : "not constrained"));

    // There’s a direct way to do this using the KinematicConstraintSet class.
    kinematic_constraints::ConstraintEvaluationResult constraint_eval_result =
            kinematic_constraint_set.decide(copied_state);
    ROS_INFO_STREAM("Test 10: Random state is "
                            << (constraint_eval_result.satisfied ? "constrained" : "not constrained"));

    while (ros::ok()){

    }
}