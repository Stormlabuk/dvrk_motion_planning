//
// Created by aleks on 01/03/2021.
//


#ifndef DVRK_MOVEIT_DVRK_MOVEIT_CLASS_H
#define DVRK_MOVEIT_DVRK_MOVEIT_CLASS_H

#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <memory>

class MoveItDVRKPlanning {

public:
    // #######################
    // ### DVRK MOVE GROUP ###
    // #######################
    int dvrk_version;                           // dvrk version, can be 1 or 2
    std::string move_group_name = "psm_arm";    // MoveIt move group name
    std::string arm_name;                       // Arm name, used to gather the dvrk ROS topics
    std::vector<geometry_msgs::Pose> waypoints; // Trajectory waypoints
    std::vector<double> tolerance_pose;         // Translational tolerance for cartesian path evaluation
    std::vector<double> tolerance_angle;        // Rotational tolerance for cartesian path evaluation
    planning_interface::MotionPlanRequest req;  // MoveIt planning request
    planning_interface::MotionPlanResponse res; // MoveIt planning response
    float max_vel_scaling_factor;
    const double jump_threshold = 0.0;
    const double eef_step = 0.001;              // max end effector step used during trajectory evaluation
    geometry_msgs::Pose home_pose;              // home pose for the robot
    geometry_msgs::PoseStamped  cart_pose;      // current cartesian dVRK pose (API v1.x)
    geometry_msgs::TransformStamped cart2_pose; // current cartesian dVRK pose (API v2.x)
    sensor_msgs::JointState joint_pose;         // current joint dVRK pose

    // #####################
    // ### ROS PUBS/SUBS ###
    // #####################

    ros::NodeHandle node_handle;
    ros::Publisher cartesian_pub;               // cartesian trajectory publisher
    ros::Publisher joint_pub;                   // joint trajectory publisher
    ros::Subscriber cp_sub;                     // DVRK arm cartesian position subscriber
    ros::Subscriber js_sub;                     // DVRK arm joint state subscriber

    // #####################
    // ### MOVEIT! SETUP ###
    // #####################

    moveit::planning_interface::MoveGroupInterface move_group = moveit::planning_interface::MoveGroupInterface(move_group_name);
    robot_model_loader::RobotModelLoader robot_model_loader = robot_model_loader::RobotModelLoader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr robot_state = std::make_shared<robot_state::RobotState>(robot_model);
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(move_group_name);
    robot_state::RobotState start_state = robot_state::RobotState(*move_group.getCurrentState()); // initial state of the robot
    planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    moveit_visual_tools::MoveItVisualTools visual_tools = moveit_visual_tools::MoveItVisualTools("psm_tool_tip_link");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // ########################
    // ### CON/DE-STRUCTORS ###
    // ########################

    explicit MoveItDVRKPlanning(std::string arm, int version = 1);

    ~MoveItDVRKPlanning();

    // #######################
    // ### SETUP FUNCTIONS ###
    // #######################


    // --- cp_callback: callback function for cartesian point position of the arm. The PoseStamped msg is copied to the
    // <cart_pose> class attribute. Works only with dVRK API v1.x
    void cp_callback(const geometry_msgs::PoseStamped msg);

    // --- cp_callback: callback function for cartesian point position of the arm. The PoseStamped msg is copied to the
    // <cart_pose> class attribute. Works only with dVRK API v2.x
    void cp2_callback(const geometry_msgs::TransformStamped msg);

    // --- cp_callback: callback function for joint state of the arm. The PoseStamped msg is copied to the <joint_pose>
    // class attribute. Works for both versions of dVRK API v1.x and v2.x.
    void js_callback(const sensor_msgs::JointState msg);

    // --- setupDVRKCartesianTrajectoryPublisher: this function sets up the publisher to the right dVRK topic depending on
    // the software version. Different dVRK versions publish on different topics and different messages. The function
    // defines and initiates the publisher for "cartesian_pub", thus requires a cartesian trajectory to be published.
    void setupDVRKCartesianTrajectoryPublisher();

    // --- setupDVRKJointTrajectoryPublisher: this function sets up the publisher to the right dVRK topic depending on
    // the software version. Different dVRK versions publish on different topics and different messages. The function
    // defines and initiates the publisher for "joint_pub", thus requires a cartesian trajectory to be published.
    void setupDVRKJointTrajectoryPublisher();

    void setupDVRKSubsribers();



    // --- setupPlanningScene: planning scene is first cleared and than populated with the robot model.
    void setupPlanningScene();

    // --- loadPlannerPlugin: class loader is used to load the planning plugin. In this specific case the planner used
    // is STOMP (https://ros-planning.github.io/moveit_tutorials/doc/stomp_planner/stomp_planner_tutorial.html)
    planning_interface::PlannerManagerPtr loadPlannerPlugin();

    // ############################
    // ### TRAJECTORY FUNCTIONS ###
    // ############################

    // --- getWaypointsVector: this function returns a fixed set of waypoints for the trajectory. The traj_ID defines
    // which waypoints to use. Available traj_ID: 'L' (left), 'R' (right), 'B' (bottom),
    std::vector<geometry_msgs::Pose> getWaypointsVector(char traj_ID);

    // --- getRandomWaypointsVector: returns a random set of <n> points. This points, defined as geometry_msg::Pose are
    // randomly evaluated from the move group forward kinematics and, thus, they are always valid. <eef_name> sets the
    // link of reference for the generated poses.
    std::vector<geometry_msgs::Pose> getRandomWaypointsVector(int n, std::string eef_name = "psm_tool_tip_link");

    // --- computeGoalConstraint: given a <goal_pose>, a kinematic constraint for the trajectory goal is evaluated and
    // it is returned from this function.
    moveit_msgs::Constraints computeGoalConstraint(geometry_msgs::Pose goal_pose);

    // --- convertJointTrajectoryCartesian(): this function converts the estimated trajectory in MoveItDVRKPlanning res
    // into a vector of poses. This function is recommended if working with dVRK v1.x
    std::vector<geometry_msgs::Pose> convertJointTrajectoryToCartesian ();

    // --- convertJointTrajectoryCartesian(): this function converts the estimated trajectory in MoveItDVRKPlanning res
    // into a vector of poses. This function is recommended if working with dVRK v1.x
    std::vector<geometry_msgs::TransformStamped> convertJointTrajectoryToCartesianStamped();

    // --- convertJointTrajectoryToJointState: this function converts the estimated trajectory in MoveItDVRKPlanning res
    // into a vector of joint states. This function works with any version of dVRK as the topic for setting a joint
    // trajectory remains the same from version 1.x to 2.x
    std::vector<sensor_msgs::JointState> convertJointTrajectoryToJointState ();

    // ---compileMotionPlanRequest: given a <goal_constraint> and a <trajecrtory> this function compiles the MoveIt!
    // planning request. Within this function the STOMP planner is called to smooth the <trajectory> evaluated before as
    // a computeCartesianPath().
    void compileMotionPlanRequest(moveit_msgs::Constraints goal_constraint, moveit_msgs::RobotTrajectory trajectory);

    // #############
    // ### UTILS ###
    // #############

    // --- displayResultTrajectory: shows the trajectory on RViz. Requires the currently used <node_handle>.
    void displayResultTrajectory();

    // --- displayWaypoints(): shows location of the defined waypoints
    void displayWaypoints();

    // --- checkWaypointsValidity: this function checks the reachability of a vector of waypoints <wp_vector>. The
    // final result and validity of the points is printed to screen.
    void checkWaypointsValidity(std::vector<geometry_msgs::Pose> wp_vector);

    // --- checkPoseValidity: this function checks the reachability of a waypoint <wp_vector>. The final result and
    // validity of the points is printed to screen.
    void checkPoseValidity(geometry_msgs::Pose);

};


#endif //DVRK_MOVEIT_DVRK_MOVEIT_CLASS_H
