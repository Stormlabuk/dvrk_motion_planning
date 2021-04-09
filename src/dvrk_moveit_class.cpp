//
// Created by aleks on 01/03/2021.
//
#include <string.h>
#include <geometry_msgs/Pose.h>
#include "dvrk_moveit_class.h"
#include <tf2_ros/buffer.h>
#include <moveit/planning_interface/planning_interface.h>
//#include <pluginlib/class_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <boost/scoped_ptr.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <stomp_moveit/stomp_planner.h>
#include <moveit/kinematic_constraints/utils.h>
#include <sstream>

#define GET_VARIABLE_NAME(Variable) (#Variable)

std::vector<geometry_msgs::Pose> MoveItDVRKPlanning::getWaypointsVector(char traj_ID) {
    //waypooints vector
    std::vector<geometry_msgs::Pose> waypoints;

    // points -> waypoints
    geometry_msgs::Pose tpose_1;
    geometry_msgs::Pose tpose_2;
    geometry_msgs::Pose tpose_3;

//    home_pose.position.x = 0.02;
//    home_pose.position.y = 0.02;
//    home_pose.position.z = -0.09;
////    home_pose.orientation.x = 1;
//
//    home_pose.orientation.x = -0.7071068;
//    home_pose.orientation.y = 0;
//    home_pose.orientation.z = 0;
//    home_pose.orientation.w = 0.7071068;

    // waypoints for left trajectory (L)
    if(traj_ID == 'L'){
        tpose_1.position.x = 0.11;
        tpose_1.position.y = 0.07;
        tpose_1.position.z = -0.07;
        tpose_1.orientation = home_pose.orientation;

        tpose_2.position.x = 0.06;
        tpose_2.position.y = 0.05;
        tpose_2.position.z = -0.05;
        tpose_2.orientation = home_pose.orientation;

        tpose_3.position.x = 0.02;
        tpose_3.position.y = 0.09;
        tpose_3.position.z = -0.09;
        tpose_3.orientation = home_pose.orientation;

    }

    if (traj_ID == 'R'){

        tpose_1.position.x = -0.08;
        tpose_1.position.y = 0.02;
        tpose_1.position.z = -0.05;
        tpose_1.orientation = home_pose.orientation;

        tpose_2.position.x = 0.0;
        tpose_2.position.y = 0.05;
        tpose_2.position.z = -0.05;
        tpose_2.orientation = home_pose.orientation;

        tpose_3.position.x = 0.06;
        tpose_3.position.y = 0.02;
        tpose_3.position.z = -0.04;
        tpose_3.orientation = home_pose.orientation;
    }

    if (traj_ID == 'B'){

        tpose_1.position.x = 0.02;
        tpose_1.position.y = 0.03;
        tpose_1.position.z = -0.03;
        tpose_1.orientation = home_pose.orientation;

        tpose_2.position.x = -0.02;
        tpose_2.position.y = 0.05;
        tpose_2.position.z = -0.05;
        tpose_2.orientation = home_pose.orientation;

        tpose_3.position.x = 0.02;
        tpose_3.position.y = 0.08;
        tpose_3.position.z = -0.09;
        tpose_3.orientation = home_pose.orientation;
    }

    if (traj_ID == 'W'){
        tpose_1.position.x = 0.07;
        tpose_1.position.y = 0.05;
        tpose_1.position.z = -0.05;
        tpose_1.orientation = home_pose.orientation;

        tpose_2.position.x = -0.02;
        tpose_2.position.y = 0.05;
        tpose_2.position.z = -0.05;
        tpose_2.orientation = home_pose.orientation;
    }

    waypoints.push_back(tpose_1);
    waypoints.push_back(tpose_2);
    if (traj_ID != 'W'){ waypoints.push_back(tpose_3);}


    return waypoints;
}

static std::vector<geometry_msgs::Pose> getSeededWaypointsVector(char traj_ID, float margin){

//    float r2 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/));

}

std::vector<geometry_msgs::Pose> MoveItDVRKPlanning::getRandomWaypointsVector(int n, std::string eef_name){

    std::vector<geometry_msgs::Pose> waypoints;

    // points -> waypoints

    for (int i = 0; i < n; i++) {
        geometry_msgs::PoseStamped tpose = move_group.getRandomPose(eef_name);
        waypoints.push_back(tpose.pose);
    }

    return waypoints;
}

planning_interface::PlannerManagerPtr MoveItDVRKPlanning::loadPlannerPlugin() {
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
//        std::string pippo = node_handle.getNamespace();
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

    return planner_instance;
}

MoveItDVRKPlanning::MoveItDVRKPlanning(std::string arm, int version){
    tolerance_pose = std::vector<double> (3,0.01);
    tolerance_angle = std::vector<double> (3,0.01);
    max_vel_scaling_factor = 0.04;
    planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");
    dvrk_version = version;
    arm_name = arm;

    home_pose.position.x = 0.02;
    home_pose.position.y = 0.02;
    home_pose.position.z = -0.09;
//    home_pose.orientation.x = 1;

    home_pose.orientation.x = -0.7071068;
    home_pose.orientation.y = 0;
    home_pose.orientation.z = 0;
    home_pose.orientation.w = 0.7071068;
}

MoveItDVRKPlanning::~MoveItDVRKPlanning()= default;

void MoveItDVRKPlanning::setupPlanningScene() {
    visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();  // clear all old markers
    visual_tools.trigger();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();
    visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
    visual_tools.trigger();
}

moveit_msgs::Constraints MoveItDVRKPlanning::computeGoalConstraint(geometry_msgs::Pose goal_pose){

    geometry_msgs::PoseStamped pose_end;
    pose_end.header.frame_id = "world";
    pose_end.pose = goal_pose;

    moveit_msgs::Constraints goal_cons =
            kinematic_constraints::constructGoalConstraints("psm_tool_tip_link", pose_end, tolerance_pose, tolerance_angle);

    return goal_cons;

}

std::vector<geometry_msgs::Pose> MoveItDVRKPlanning::convertJointTrajectoryToCartesian () {

    std::vector<geometry_msgs::Pose> pose_trajectory;

    if (dvrk_version == 1) {
        moveit_msgs::MotionPlanResponse response;
        res.getMessage(response);

        for (int i = 0; i < response.trajectory.joint_trajectory.points.size(); i++) {
            std::vector<double> joint_values;

            for (int k = 0; k < response.trajectory.joint_trajectory.joint_names.size(); k++) {
                joint_values.push_back(response.trajectory.joint_trajectory.points.at(i).positions.at(k));
            }

            //initialize joint values
            robot_state->setJointGroupPositions(joint_model_group->getName(), joint_values);
            const Eigen::Affine3d &link_pose = robot_state->getGlobalLinkTransform("psm_tool_tip_link");
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

            pose_trajectory.push_back(tp);
        }

        return pose_trajectory;
    } else { ROS_ERROR("DVRK version >= 2. Please, use convertJointTrajectoryToCartesianStamped instead.");
                return pose_trajectory;}
}

std::vector<geometry_msgs::TransformStamped> MoveItDVRKPlanning::convertJointTrajectoryToCartesianStamped() {
    if(dvrk_version == 2){
        std::vector<geometry_msgs::TransformStamped> pose_trajectory;
        moveit_msgs::MotionPlanResponse response;
        res.getMessage(response);

        for (int i = 0; i<response.trajectory.joint_trajectory.points.size(); i++) {
            std::vector<double> joint_values;

            for (int k = 0; k<response.trajectory.joint_trajectory.joint_names.size(); k++) {
                joint_values.push_back(response.trajectory.joint_trajectory.points[i].positions[k]);}

            //initialize joint values
            robot_state->setJointGroupPositions(joint_model_group->getName(), joint_values);
            const Eigen::Affine3d &link_pose = robot_state->getGlobalLinkTransform("psm_tool_tip_link");
            Eigen::Vector3d cartesian_position = link_pose.translation();
            Eigen::Matrix3d link_orientation = link_pose.rotation();
            Eigen::Quaterniond rot_quat(link_orientation);

            // populate Pose Message
            geometry_msgs::TransformStamped tp;
            tp.header = response.trajectory.joint_trajectory.header;
            tp.child_frame_id = "psm_tool_tip_link";
            tp.transform.translation.x = cartesian_position.x();
            tp.transform.translation.y = cartesian_position.y();
            tp.transform.translation.z = cartesian_position.z();
            tp.transform.rotation.w = rot_quat.w();
            tp.transform.rotation.x = rot_quat.x();
            tp.transform.rotation.y = rot_quat.y();
            tp.transform.rotation.z = rot_quat.z();

            pose_trajectory.push_back(tp);}

        return pose_trajectory;}
}

void MoveItDVRKPlanning::cp_callback(const geometry_msgs::PoseStamped msg){
    cart_pose = msg;
}

void MoveItDVRKPlanning::cpl_callback(const geometry_msgs::PoseStamped msg){
    cart_local_pose = msg;
}

void MoveItDVRKPlanning::cp2_callback(const geometry_msgs::TransformStamped msg){
    cart2_pose = msg;
}

void MoveItDVRKPlanning::js_callback(const sensor_msgs::JointState msg){
    joint_pose = msg;
}

void MoveItDVRKPlanning::bf_callback(const geometry_msgs::Pose msg){
    base_frame = msg;
}

void MoveItDVRKPlanning::setupDVRKCartesianTrajectoryPublisher(){

    std::stringstream topic_name;

    if (dvrk_version == 1) {
        topic_name << "/dvrk/" << arm_name << "/set_position_goal_cartesian";
        cartesian_pub = node_handle.advertise<geometry_msgs::Pose>(topic_name.str(), 1000);
        topic_name.str("");
    }
    else if (dvrk_version==2){
        topic_name << "/" << arm_name << "/setpoint_cp";
        cartesian_pub = node_handle.advertise<geometry_msgs::TransformStamped>(topic_name.str(), 1000);}
    else{ROS_ERROR("Unknown dVRK version! DVRK version (MoveItDVRKPlanning.dvrk_version) can be either 1 or 2.");}

}

void MoveItDVRKPlanning::setupDVRKSubsribers(){

    std::stringstream topic_name;
    topic_name << "/dvrk/" << arm_name << "/get_base_frame";
    bf_sub = node_handle.subscribe(topic_name.str(), 1000, &MoveItDVRKPlanning::bf_callback, this);
    std::cout << topic_name.str() << std::endl;
    topic_name.str("");

    if (dvrk_version == 1) {
        topic_name << "/dvrk/" << arm_name << "/position_cartesian_current";
        std::cout << topic_name.str() << std::endl;
        cp_sub = node_handle.subscribe(topic_name.str(), 1000, &MoveItDVRKPlanning::cp_callback,this);
        topic_name.str("");
        topic_name << "/dvrk/" << arm_name << "/position_cartesian_local_current";
        std::cout << topic_name.str() << std::endl;
        cpl_sub = node_handle.subscribe(topic_name.str(), 1000, &MoveItDVRKPlanning::cpl_callback,this);
        topic_name.str("");
        topic_name << "/dvrk/" << arm_name << "/position_joint_current";
        js_sub = node_handle.subscribe(topic_name.str(), 1000, &MoveItDVRKPlanning::js_callback, this);
    }

    else if(dvrk_version==2){
        topic_name << "/" << arm_name << "/measured_cp";
        cp_sub = node_handle.subscribe(topic_name.str(), 1000, &MoveItDVRKPlanning::cp2_callback,this);
        topic_name.str("");
        topic_name << "/" << arm_name << "/measured_js";
        js_sub = node_handle.subscribe(topic_name.str(), 1000, &MoveItDVRKPlanning::js_callback, this);
    }
    else{ROS_ERROR("Unknown dVRK version! DVRK version (MoveItDVRKPlanning.dvrk_version) can be either 1 or 2.");
    }
}

void MoveItDVRKPlanning::setupDVRKJointTrajectoryPublisher() {
    std::stringstream topic_name;

    if (dvrk_version == 1){
        topic_name << "/dvrk/" << arm_name << "/set_position_goal_joint";
        joint_pub = node_handle.advertise<sensor_msgs::JointState>(topic_name.str(), 1000);}

    else if (dvrk_version==2){
        topic_name << "/" << arm_name << "/setpoint_js";
        joint_pub = node_handle.advertise<sensor_msgs::JointState>(topic_name.str(), 1000);}
    else{ROS_ERROR("Unknown dVRK version! DVRK version (MoveItDVRKPlanning.dvrk_version) can be either 1 or 2.");}
}

std::vector<sensor_msgs::JointState> MoveItDVRKPlanning::convertJointTrajectoryToJointState (){

    std::vector<sensor_msgs::JointState> joint_trajectory;
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    for (int i = 0; i < response.trajectory.joint_trajectory.points.size(); i++) {

        sensor_msgs::JointState joint_pose;
        joint_pose.header = response.trajectory.joint_trajectory.header;
        joint_pose.name = response.trajectory.joint_trajectory.joint_names;
        joint_pose.position = response.trajectory.joint_trajectory.points[i].positions;
        joint_pose.velocity = response.trajectory.joint_trajectory.points[i].velocities;

        joint_trajectory.push_back(joint_pose);
    }

    return joint_trajectory;
}

void MoveItDVRKPlanning::compileMotionPlanRequest(moveit_msgs::Constraints goal_constraint, moveit_msgs::RobotTrajectory trajectory){

    req.group_name = move_group_name;
    req.goal_constraints.push_back(goal_constraint);
    req.allowed_planning_time = 10.;
    req.trajectory_constraints = stomp_moveit::StompPlanner::encodeSeedTrajectory(trajectory.joint_trajectory);
    moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);

}

void MoveItDVRKPlanning::displayResultTrajectory(){
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
    visual_tools.publishAxisLabeled(MoveItDVRKPlanning::convertMatrixToPose(MoveItDVRKPlanning::invertHomoMatrix(convertPoseToMatrix(base_frame))), "camera_frame");
    for (int i = 0; i < waypoints.size(); i++){
        std::ostringstream goal_n;
        goal_n << "goal_" << i << std::endl;
        visual_tools.publishAxisLabeled(waypoints.at(i), goal_n.str());
        goal_n.clear();
    }
    visual_tools.trigger();

}

void MoveItDVRKPlanning::displayWaypoints(){
    for (int i = 0; i < waypoints.size(); i++){
        std::ostringstream goal_n;
        goal_n << "goal_" << i << std::endl;
        visual_tools.publishAxisLabeled(waypoints.at(0), goal_n.str());
    }
}

void MoveItDVRKPlanning::checkWaypointsValidity(std::vector<geometry_msgs::Pose> wp_vector){

    ROS_INFO("Waypoints pose validity check:");

    for (int i = 0; i< wp_vector.size(); i++){
        if(move_group.setJointValueTarget(wp_vector.at(i),"psm_tool_tip_link")){
            ROS_INFO("Waypoints #%d: VALID", i);
        } else{ ROS_ERROR("!!! Waypoints #%d: INVALID",i);}
    }

}

Eigen::Matrix4d MoveItDVRKPlanning::invertHomoMatrix (Eigen::Matrix4d mat){

    Eigen::Matrix4d tf_inv;
    Eigen::Vector4d tf_pos;

    tf_pos = -1 * mat.col(3);
    tf_pos(3) = 1;


    tf_inv.block(0,0,3,3) = mat.block(0,0,3,3).transpose();
    tf_inv(3,3) = 1;
    // tf_inv.col(3) = tf_inv.block(0,0,3,3) * mat.block(0,3,3,1);
    tf_inv.col(3) = tf_inv * tf_pos;
//    tf_inv(3,3) = 1;

    std::cout << "Input Matrix: \n" << mat << std::endl;
    std::cout << "tf_pose Matrix: \n" << tf_pos << std::endl;
    std::cout << "tf_inv Matrix: \n" << tf_inv << std::endl;

    return tf_inv;
}

void MoveItDVRKPlanning::checkPoseValidity(geometry_msgs::Pose pose){

    ROS_INFO("Pose validity check:");
//    string pose_name = GET_VARIABLE_NAME(pose);
        if(move_group.setJointValueTarget(pose,"psm_tool_tip_link")){
            ROS_INFO("Pose %s: VALID", GET_VARIABLE_NAME(pose));
        } else{ ROS_ERROR("!!! Pose %s: INVALID",GET_VARIABLE_NAME(pose));}

}

Eigen::Matrix4d MoveItDVRKPlanning::convertPoseToMatrix(geometry_msgs::Pose pose){

    Eigen::Matrix4d res;
    Eigen::Vector4d V;
    V(0) = 0;
    V(1) = 0;
    V(2) = 0;
    V(3) = 1;
    Eigen::MatrixXd RT(3,4);

    Eigen::Vector3d T(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

    Eigen::Matrix3d R = q.toRotationMatrix();

    RT << R,T;
    res.block(0,0,3,4) = RT;
    res.row(3) = V;

    return res;
}

geometry_msgs::Pose MoveItDVRKPlanning::convertMatrixToPose (Eigen::Matrix4d mat){
    geometry_msgs::Pose res;

    res.position.x = mat(0,3);
    res.position.y = mat(1,3);
    res.position.z = mat(2,3);

    Eigen::Matrix3d rot_mat;
    rot_mat = mat.block(0,0,3,3);
    Eigen::Quaterniond quat_rot(rot_mat);

    res.orientation.w = quat_rot.w();
    res.orientation.x = quat_rot.x();
    res.orientation.y = quat_rot.y();
    res.orientation.z = quat_rot.z();

    return res;
}

std::vector<geometry_msgs::Pose> MoveItDVRKPlanning::transformTrajectory(std::vector<geometry_msgs::Pose> traj, geometry_msgs::Pose base_frame){
    std::vector<geometry_msgs::Pose> res;
    geometry_msgs::PoseStamped trans;

    for(int i = 0; i < traj.size(); i++) {
        Eigen::Matrix4d pos_mat = convertPoseToMatrix(traj[i]);
        Eigen::Matrix4d bf_mat = convertPoseToMatrix(base_frame);

        Eigen::Matrix4d trans_mat =  bf_mat * pos_mat;

        geometry_msgs::Pose trans_pose = convertMatrixToPose(trans_mat);

        res.push_back(trans_pose);
    }

    return  res;
}
