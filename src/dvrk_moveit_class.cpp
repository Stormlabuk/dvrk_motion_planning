//
// Created by aleks on 01/03/2021.
//
#include <string.h>
#include <geometry_msgs/Pose.h>
#include "dvrk_moveit_class.h"

std::vector<geometry_msgs::Pose> MoveItDVRK::getWaypointsVector(char traj_ID) {
    //waypooints vector
    std::vector<geometry_msgs::Pose> waypoints;

    // points -> waypoints
    geometry_msgs::Pose tpose_1;
    geometry_msgs::Pose tpose_2;
    geometry_msgs::Pose tpose_3;

    // waypoints for left trajectory (L)
    if(traj_ID = 'L'){
        tpose_1.position.x = 0.11;
        tpose_1.position.y = 0.07;
        tpose_1.position.z = -0.07;
        tpose_1.orientation.w = 0.4;

        tpose_2.position.x = 0.06;
        tpose_2.position.y = 0.05;
        tpose_2.position.z = -0.05;
        tpose_2.orientation.w = 1.0;

        tpose_3.position.x = 0.02;
        tpose_3.position.y = 0.09;
        tpose_3.position.z = -0.09;
        tpose_3.orientation.w = 1;
    }

    if (traj_ID = 'R'){

        tpose_1.position.x = -0.08;
        tpose_1.position.y = 0.02;
        tpose_1.position.z = -0.05;
        tpose_1.orientation.w = 0.4;

        tpose_2.position.x = 0.0;
        tpose_2.position.y = 0.05;
        tpose_2.position.z = -0.05;
        tpose_2.orientation.w = 1.0;

        tpose_3.position.x = 0.06;
        tpose_3.position.y = 0.02;
        tpose_3.position.z = -0.04;
        tpose_3.orientation.w = 1;
    }

    if (traj_ID = 'B'){

        tpose_1.position.x = 0.01;
        tpose_1.position.y = 0.02;
        tpose_1.position.z = -0.02;
        tpose_1.orientation.w = 0.4;

        tpose_2.position.x = -0.02;
        tpose_2.position.y = 0.05;
        tpose_2.position.z = -0.05;
        tpose_2.orientation.w = 1.0;

        tpose_3.position.x = 0.02;
        tpose_3.position.y = 0.08;
        tpose_3.position.z = -0.09;
        tpose_3.orientation.w = 1;
    }


    waypoints.push_back(tpose_1);
    waypoints.push_back(tpose_2);
    waypoints.push_back(tpose_3);

    return waypoints;
}