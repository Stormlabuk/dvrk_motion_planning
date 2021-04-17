#include <ros/ros.h>
#include <dvrk_moveit_class.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
// #include <ncurses.h>

// MoveIt!
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf2_msgs/TFMessage.h>

Eigen::Matrix4d tf_cam_aruco;
//const Eigen::Vector4d p1_cb(0.066, 0.035, 0.03, 1);
//const Eigen::Vector4d p2_cb(0.105, 0.004, 0.05, 1);
//const Eigen::Vector4d p3_cb(0.08, -0.022, 0.035, 1);
//const Eigen::Vector4d p4_cb(0.05, -0.01, 0.04, 1);
//const Eigen::Vector4d p_offset(0, 0, 0.005, 0);
//
//const Eigen::Vector4d p1_cb_offset(-0.005, 0.015, 0.012, 0);
//const Eigen::Vector4d p2_cb_offset(-0.01, 0.02, 0.01, 0);
//const Eigen::Vector4d p3_cb_offset(0.002, 0.017, 0.007, 0);
//const Eigen::Vector4d p4_cb_offset(0.0, 0.0, 0.0, 0);

const Eigen::Vector4d p1_cb(0.066, 0.035, 0.03, 1);
const Eigen::Vector4d p2_cb(0.105, 0.004, 0.05, 1);
const Eigen::Vector4d p3_cb(0.08, -0.022, 0.035, 1);
const Eigen::Vector4d p4_cb(0.05, -0.01, 0.04, 1);
const Eigen::Vector4d p_offset(0, 0, 0.005, 0);

Eigen::Vector4d p1_aruco;
Eigen::Vector4d p2_aruco;
Eigen::Vector4d p3_aruco;
Eigen::Vector4d p4_aruco;

Eigen::Vector4d p1_cam;
Eigen::Vector4d p2_cam;
Eigen::Vector4d p3_cam;
Eigen::Vector4d p4_cam;

geometry_msgs::Pose pose_1;
geometry_msgs::Pose pose_2;
geometry_msgs::Pose pose_3;
geometry_msgs::Pose pose_4;

Eigen::Matrix4d tf_aruco_cb;
Eigen::Matrix4d tf_des_cb;
Eigen::Matrix4d tf_des_cam;
bool tf_recvd = false;

void init_tf_aruco_tfs(){
// tf_aruco_cb << 1, 0, 0, 0, 0, 0.5, 0.866025, -0.04596, 0, -0.866025, 0.5, -0.0230718, 0, 0, 0, 1;
    tf_aruco_cb << 1, 0, 0, -0.065, 0, 1, 0, 0.056, 0, 0, 1, 0, 0, 0, 0, 1;
    tf_des_cb << -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1;

    tf_des_cam = tf_cam_aruco * tf_aruco_cb * tf_des_cb;
    std::cout << "tf_aruco_cb\n" << tf_aruco_cb << std::endl;
//
//    p1_aruco = tf_aruco_cb * (p1_cb + p_offset - p1_cb_offset);
//    p2_aruco = tf_aruco_cb * (p2_cb + p_offset - p2_cb_offset);
//    p3_aruco = tf_aruco_cb * (p3_cb + p_offset - p3_cb_offset);
//    p4_aruco = tf_aruco_cb * (p4_cb + p_offset - p4_cb_offset);

    p1_aruco = tf_aruco_cb * (p1_cb + p_offset);
    p2_aruco = tf_aruco_cb * (p2_cb + p_offset);
    p3_aruco = tf_aruco_cb * (p3_cb + p_offset);
    p4_aruco = tf_aruco_cb * (p4_cb + p_offset);

//    p3_aruco.setZero();
//    p3_aruco(2) = 0.05;
//    p3_aruco(3) = 1;

    p1_cam = tf_cam_aruco * p1_aruco;
    p2_cam = tf_cam_aruco * p2_aruco;
    p3_cam = tf_cam_aruco * p3_aruco;
    p4_cam = tf_cam_aruco * p4_aruco;

    // Eigen::Quaterniond q_des_cam(Eigen::Matrix3d(tf_des_cam.block(0,0,3,3)));

    pose_1.position.x = p1_cam(0);
    pose_1.position.y = p1_cam(1);
    pose_1.position.z = p1_cam(2);

//    pose_1.orientation.x = q_des_cam.x();
//    pose_1.orientation.x = q_des_cam.y();
//    pose_1.orientation.x = q_des_cam.z();
//    pose_1.orientation.x = q_des_cam.w();

    pose_2.position.x = p2_cam(0);
    pose_2.position.y = p2_cam(1);
    pose_2.position.z = p2_cam(2);

    pose_3.position.x = p3_cam(0);
    pose_3.position.y = p3_cam(1);
    pose_3.position.z = p3_cam(2);

    pose_4.position.x = p4_cam(0);
    pose_4.position.y = p4_cam(1);
    pose_4.position.z = p4_cam(2);

    std::cout << "p1\n" << p1_aruco << "\np2\n" << p2_aruco << "\np3\n" << p3_aruco << "\np4\n" << p4_aruco << std::endl;
    std::cout << "p1\n" << p1_cam<< "\np2\n" << p2_cam << "\np3\n" << p3_cam << "\np4\n" << p4_cam << std::endl;
}

void aruco_callback(tf2_msgs::TFMessage msg){
    if(msg.transforms[0].child_frame_id == "tf_cam_cbar"){
        Eigen::Matrix4d mat;
        mat(0,3) = msg.transforms[0].transform.translation.x;
        mat(1,3) = msg.transforms[0].transform.translation.y;
        mat(2,3) = msg.transforms[0].transform.translation.z;
        mat(3,3) = 1;

        Eigen::Quaterniond q(msg.transforms[0].transform.rotation.w, msg.transforms[0].transform.rotation.x,
                           msg.transforms[0].transform.rotation.y, msg.transforms[0].transform.rotation.z);
        mat.block(0,0,3,3) = q.toRotationMatrix();
        tf_cam_aruco = mat;

//        std::cout << "TF_cam_aruco" << "\n" << mat << std::endl;
//        std::cout << msg.transforms[0].transform.translation << std::endl;

        tf_recvd = true;
    }
}

int main(int argc, char** argv) {


    ros::init(argc, argv, "spline_planning");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    MoveItDVRKPlanning mid("PSM1", 1);

    ros::Subscriber aruco_sub = mid.node_handle.subscribe("/tf", 1000, aruco_callback);

    // ### SETUP PUBLISHERS ###
    mid.setupDVRKCartesianTrajectoryPublisher();
    mid.setupDVRKJointTrajectoryPublisher();
    mid.setupDVRKSubsribers();

    ros::Rate rlong(1);

    while(!tf_recvd){
        std::cout << "Wating for TF_CAM_ARUCO" << std::endl;
        rlong.sleep();
    }

    init_tf_aruco_tfs();

    // TODO: Unsubscribe from Aruco topic
    std::cout << tf_cam_aruco << std::endl;

    while (mid.cart_pose.pose.position.x == 0 && mid.base_frame.position.x == 0){
        ROS_INFO("Waiting...");
        rlong.sleep();
    }

    // ### LOAD PLANNER PLUGIN ###
    planning_interface::PlannerManagerPtr planner_instance = mid.loadPlannerPlugin();

    // ### SETUP PLANNING SCENE ###
    mid.setupPlanningScene();

    // ### DEFINE AND VALIDATE WAYPOINTS ###
//    mid.waypoints = mid.getWaypointsVector('B');
//    mid.move_group.setPoseReferenceFrame("psm_tool_tip_link");
    ROS_INFO_STREAM("Planning using as reference frame: " << mid.move_group.getPoseReferenceFrame());

    mid.checkPoseValidity(mid.cart_local_pose.pose);

    std::vector<geometry_msgs::Pose> waypoints_;
    waypoints_.push_back(pose_4);
//    waypoints_.push_back(pose_1);
    waypoints_.push_back(pose_3);
    waypoints_.push_back(pose_2);

    geometry_msgs::Pose inv_pose = MoveItDVRKPlanning::convertMatrixToPose(MoveItDVRKPlanning::invertHomoMatrix(mid.convertPoseToMatrix(mid.base_frame)));
    std::vector<geometry_msgs::Pose> waypoints_t = mid.transformTrajectory(waypoints_, inv_pose);

    mid.waypoints = waypoints_t;
    rlong.sleep();

    std::cout << "tf_des_cam\n" << MoveItDVRKPlanning::convertPoseToMatrix(inv_pose) * tf_des_cam << std::endl;

    for(int i =0; i < mid.waypoints.size(); i++){
    mid.waypoints.at(i).orientation = mid.cart_local_pose.pose.orientation;}
//    mid.waypoints.at(1).orientation = mid.cart_local_pose.pose.orientation;
//    mid.waypoints.at(2).orientation = mid.cart_local_pose.pose.orientation;
//    mid.waypoints.at(3).orientation = mid.cart_local_pose.pose.orientation;

//    while(true){
//        std::cout << mid.cart_local_pose.pose << std::endl;
//        rlong.sleep();
//    }

    // geometry_msgs::Pose p0 = mid.cart_local_pose.pose;
    // p0.orientation = waypoints_t.at(0).orientation;
//    waypoints_t.at(0).position = mid.cart_local_pose.pose.position;
//    waypoints_t.at(0).position.z -= 0.03;

//    std::cout << mid.waypoints.at(0) << std::endl;
//    std::cout << waypoints_t.at(1) << std::endl;
//    mid.waypoints = waypoints_t;
//    std::cout << waypoints_t.at(2) << std::endl;
//    std::cout << waypoints_t.at(3) << std::endl;

//    mid.visual_tools.publishAxisLabeled(waypoints_t.at(0), "Culo1");
//    mid.visual_tools.publishAxisLabeled(waypoints_t.at(1), "Culo2");
//    mid.visual_tools.publishAxisLabeled(waypoints_t.at(2), "Culo3");
//    mid.visual_tools.publishAxisLabeled(waypoints_t.at(3), "Culo4");

//    mid.visual_tools.trigger();
//
//    while(true){
//        rlong.sleep();
//    }

    mid.checkWaypointsValidity(mid.waypoints);

    // SETUP INITIAL STATE
    mid.start_state.setFromIK(mid.joint_model_group, mid.cart_local_pose.pose); // set start state as home_pose
    mid.move_group.setStartState(mid.start_state);
    mid.move_group.setMaxVelocityScalingFactor(mid.max_vel_scaling_factor);

    // ### EVALUATE CARTESIAN PATH TO SMOOTH WITH STOMP ###
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = mid.move_group.computeCartesianPath(mid.waypoints, mid.eef_step, mid.jump_threshold, trajectory) * 100;

    // ### DEFINE GOAL POSE AND COMPILE MOTION PLAN REQUEST ###
    moveit_msgs::Constraints pose_goal_end = mid.computeGoalConstraint(mid.waypoints.at(waypoints_t.size()-1));
    mid.compileMotionPlanRequest(pose_goal_end, trajectory);

    // SOLVE REQUEST
    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(mid.planning_scene, mid.req, mid.res.error_code_);
    context->setMotionPlanRequest(mid.req);
    context->solve(mid.res);

//    std::vector<sensor_msgs::JointState> joint_trajectory = mid.convertJointTrajectoryToJointState();
    std::vector<geometry_msgs::Pose> pose_trajectory = mid.convertJointTrajectoryToCartesian();
    std::vector<geometry_msgs::Pose> pose_trajectory_trans = mid.transformTrajectory(pose_trajectory, mid.base_frame);

    Eigen::Matrix4d tf_psm1_cam = MoveItDVRKPlanning::convertPoseToMatrix(inv_pose);
    std::cout << "tf_psm1_cam" << std::endl;
    std::cout << tf_psm1_cam << std::endl;
    Eigen::Matrix4d tf_psm1_cb = tf_psm1_cam * tf_cam_aruco * tf_aruco_cb;
    Eigen::Matrix4d tf_cb_psm1 = MoveItDVRKPlanning::invertHomoMatrix(tf_psm1_cb);
    std::cout << "tf_cb_psm1" << std::endl;
    std::cout << tf_cb_psm1 << std::endl;

    // ### SHOW RESULT TRAJECTORY ###
    mid.displayResultTrajectory();
    spinner.stop();
    std::cout << "Counting down..." << std::endl;
    std:: cout << "3" << std::endl;
    // beep();
    std::cout << '\b';
    usleep(1e6);
    std:: cout << "2" << std::endl;
    // beep();
    usleep(1e6);
    std:: cout << "1" << std::endl;
    std::cout << '\b';
    // beep();
    usleep(1e6);
    std:: cout << "Go" << std::endl;
    std::cout << '\b';

    // ### STOP SPINNER AND DEFINE NEW ROS SPINNER ###
    ros::Rate r(20);

    if(fraction > 30) {
        ROS_INFO("!!! Planning successful: %.03f percent of the trajectory is followed.", fraction);
        ROS_INFO("Publishing trajectory of %d points", (int) pose_trajectory.size());

        while (ros::ok()) {
            for (int i = 0; i < pose_trajectory_trans.size() - 1; i++) {
                Eigen::Vector3d v1(pose_trajectory_trans.at(i).position.x, pose_trajectory_trans.at(i).position.y, pose_trajectory_trans.at(i).position.z);
                Eigen::Vector3d v2(pose_trajectory_trans.at(i+1).position.x, pose_trajectory_trans.at(i+1).position.y, pose_trajectory_trans.at(i+1).position.z);
                std::cout << i << " out of " << pose_trajectory_trans.size()-2 << std::endl;
                // std::cout << (v1 - v2).squaredNorm() << std::endl;
                Eigen::Vector4d v1_ (v1(0),v1(1),v1(2),1);
                Eigen::Vector4d v1cazzo = tf_cb_psm1 * v1_;
                std::cout << v1cazzo << std::endl;
                std::cout << "------------" << std::endl;

                 mid.cartesian_pub.publish(pose_trajectory_trans.at(i));
                // mid.joint_pub.publish(joint_trajectory.at(i));

                ros::spinOnce();
                r.sleep();
            }
            break;
        }
        ROS_INFO("Trajectory published and executed. Shutting down node...");
    }
    else{
        ROS_ERROR("Trajectory is not feasible: %.03f of points are not reachable", 100-fraction);
    }

    return 0;
}

