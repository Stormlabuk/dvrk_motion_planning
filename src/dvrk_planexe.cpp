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

//geometry_msgs::PoseStamped cart;
//
//void cp_callback(const geometry_msgs::PoseStamped msg){
//    cart = msg;
//}

int main(int argc, char** argv) {

    ros::init(argc, argv, "spline_planning");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    MoveItDVRKPlanning mid("PSM1", 1);

    // ### SETUP PUBLISHERS ###
    mid.setupDVRKCartesianTrajectoryPublisher();
    mid.setupDVRKJointTrajectoryPublisher();
    mid.setupDVRKSubsribers();

    while (mid.cart_pose.pose.position.x == 0){ROS_INFO("Waiting...");}

    // ### LOAD PLANNER PLUGIN ###
    planning_interface::PlannerManagerPtr planner_instance = mid.loadPlannerPlugin();

    // ### SETUP PLANNING SCENE ###
    mid.setupPlanningScene();

    // ### DEFINE AND VALIDATE WAYPOINTS ###
    mid.waypoints = mid.getWaypointsVector('B');
//    mid.move_group.setPoseReferenceFrame("psm_tool_tip_link");
    ROS_INFO_STREAM("Planning using as reference frame: " << mid.move_group.getPoseReferenceFrame());

//    geometry_msgs::Pose wp1;
//    wp1.position.x = -0.02;
//    wp1.position.y = -0.01;
//    wp1.position.z = 0.02;
//    wp1.orientation = mid.home_pose.orientation;
//
//    geometry_msgs::Pose wp2;
//    wp2.position.x = 0;
//    wp2.position.y = 0;
//    wp2.position.z = 0.06;
//
//    mid.waypoints.push_back(wp1);
//    mid.waypoints.push_back(wp2);

    mid.checkPoseValidity(mid.cart_local_pose.pose);
    geometry_msgs::Pose tpose_1;
    geometry_msgs::Pose tpose_2;
    geometry_msgs::Pose tpose_3;

//    tpose_1.position.x = 0.0;
//    tpose_1.position.y = 0.0;
//    tpose_1.position.z = 0.09;
    tpose_1.position = mid.cart_pose.pose.position;
    tpose_1.position.x = 0.0;
    tpose_1.position.y = 0.0;
    tpose_1.position.z = 0.08;
    tpose_1.orientation = mid.cart_pose.pose.orientation;
//    tpose_1.orientation = mid.cart_local_pose.pose.orientation;

//    tpose_2.position.x = 0.0;
//    tpose_2.position.y = 0.0;
//    tpose_2.position.z = 0.11;
    tpose_2.position = tpose_1.position;
    tpose_2.position.x = 0.0;
    tpose_2.position.y = 0.0;
    tpose_2.position.z += 0.08;
    tpose_2.orientation = mid.cart_pose.pose.orientation;

    tpose_3.position = tpose_2.position;
    tpose_3.position.x = 0.0;
    tpose_3.position.y = 0.0;
    tpose_3.position.z -= 0.08;
    tpose_3.orientation = mid.cart_pose.pose.orientation;

//    tpose_3.position = tpose_1.position;
//    tpose_3.position.x += 0.02;
//    tpose_3.orientation = mid.cart_pose.pose.orientation;

    std::vector<geometry_msgs::Pose> waypoints_;


    waypoints_.push_back(tpose_1);
    waypoints_.push_back(tpose_2);
    waypoints_.push_back(tpose_3);

    geometry_msgs::Pose inv_pose = MoveItDVRKPlanning::convertMatrixToPose(MoveItDVRKPlanning::invertHomoMatrix(mid.convertPoseToMatrix(mid.base_frame)));
    std::vector<geometry_msgs::Pose> waypoints_t = mid.transformTrajectory(waypoints_, inv_pose);

    waypoints_t.at(0).orientation = mid.cart_local_pose.pose.orientation;
    waypoints_t.at(1).orientation = mid.cart_local_pose.pose.orientation;
    waypoints_t.at(2).orientation = mid.cart_local_pose.pose.orientation;

    std::cout << waypoints_t.at(0) << std::endl;
    std::cout << waypoints_t.at(1) << std::endl;
    std::cout << waypoints_t.at(2) << std::endl;

//    std::vector<geometry_msgs::Pose> startpoint_;

//    startpoint_.push_back(mid.cart_local_pose.pose);
//    mid.checkWaypointsValidity(startpoint_);

//    std::cout << (mid.cart_local_pose.pose.position.x - waypoints_t.at(0).position.x) << "\n"
//            << (mid.cart_local_pose.pose.position.y - waypoints_t.at(0).position.y) << "\n"
//            << (mid.cart_local_pose.pose.position.z - waypoints_t.at(0).position.z) << "\n"
//            << (mid.cart_local_pose.pose.orientation.x - waypoints_t.at(0).orientation.x) << "\n"
//            << (mid.cart_local_pose.pose.orientation.y - waypoints_t.at(0).orientation.y) << "\n"
//            << (mid.cart_local_pose.pose.orientation.z - waypoints_t.at(0).orientation.z) << "\n"
//            << (mid.cart_local_pose.pose.orientation.w - waypoints_t.at(0).orientation.w) << "\n" << std::endl;
//
//    std::cout << (mid.cart_local_pose.pose.position.x - waypoints_t.at(1).position.x) << "\n"
//              << (mid.cart_local_pose.pose.position.y - waypoints_t.at(1).position.y) << "\n"
//              << (mid.cart_local_pose.pose.position.z - waypoints_t.at(1).position.z) << "\n"
//              << (mid.cart_local_pose.pose.orientation.x - waypoints_t.at(1).orientation.x) << "\n"
//              << (mid.cart_local_pose.pose.orientation.y - waypoints_t.at(1).orientation.y) << "\n"
//              << (mid.cart_local_pose.pose.orientation.z - waypoints_t.at(1).orientation.z) << "\n"
//              << (mid.cart_local_pose.pose.orientation.w - waypoints_t.at(1).orientation.w) << std::endl;

    mid.checkWaypointsValidity(waypoints_t);

    // SETUP INITIAL STATE
    mid.start_state.setFromIK(mid.joint_model_group, mid.cart_local_pose.pose); // set start state as home_pose
    mid.move_group.setStartState(mid.start_state);
    mid.move_group.setMaxVelocityScalingFactor(mid.max_vel_scaling_factor);

    // ### EVALUATE CARTESIAN PATH TO SMOOTH WITH STOMP ###
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = mid.move_group.computeCartesianPath(waypoints_t, mid.eef_step, mid.jump_threshold, trajectory) * 100;

    // ### DEFINE GOAL POSE AND COMPILE MOTION PLAN REQUEST ###
    moveit_msgs::Constraints pose_goal_end = mid.computeGoalConstraint(waypoints_t.at(waypoints_t.size()-1));
    mid.compileMotionPlanRequest(pose_goal_end, trajectory);

    // SOLVE REQUEST
    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(mid.planning_scene, mid.req, mid.res.error_code_);
    context->setMotionPlanRequest(mid.req);
    context->solve(mid.res);

//    std::vector<sensor_msgs::JointState> joint_trajectory = mid.convertJointTrajectoryToJointState();
    std::vector<geometry_msgs::Pose> pose_trajectory = mid.convertJointTrajectoryToCartesian();
    std::vector<geometry_msgs::Pose> pose_trajectory_trans = mid.transformTrajectory(pose_trajectory, mid.base_frame);

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

    if(fraction > 95) {
        ROS_INFO("!!! Planning successful: %.03f percent of the trajectory is followed.", fraction);
        ROS_INFO("Publishing trajectory of %d points", (int) pose_trajectory.size());

        while (ros::ok()) {
            for (int i = 0; i < pose_trajectory_trans.size() - 1; i++) {
                Eigen::Vector3d v1(pose_trajectory_trans.at(i).position.x, pose_trajectory_trans.at(i).position.y, pose_trajectory_trans.at(i).position.z);
                Eigen::Vector3d v2(pose_trajectory_trans.at(i+1).position.x, pose_trajectory_trans.at(i+1).position.y, pose_trajectory_trans.at(i+1).position.z);
                std::cout << i << " out of " << pose_trajectory_trans.size()-2 << std::endl;
                // std::cout << (v1 - v2).squaredNorm() << std::endl;
                std::cout << v1 << std::endl;

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

