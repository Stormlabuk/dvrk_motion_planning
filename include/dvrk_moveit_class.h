//
// Created by aleks on 01/03/2021.
//


#ifndef DVRK_MOVEIT_DVRK_MOVEIT_CLASS_H
#define DVRK_MOVEIT_DVRK_MOVEIT_CLASS_H

#include <stdio.h>
#include <geometry_msgs/Pose.h>

class MoveItDVRK {

public:
    std::vector<geometry_msgs::Pose> getWaypointsVector(char traj_ID);
};


#endif //DVRK_MOVEIT_DVRK_MOVEIT_CLASS_H
