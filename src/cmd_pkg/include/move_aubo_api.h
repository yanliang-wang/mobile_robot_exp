//
// Created by wang on 12/6/19.
//

#ifndef SRC_MOVE_AUBO_API_H
#define SRC_MOVE_AUBO_API_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>


void print_aubo_state(moveit::planning_interface::MoveGroupInterface &move_group);

void move_by_joint(moveit::planning_interface::MoveGroupInterface &move_group,
                   const std::vector<double> &target_joint);

void move_by_coordinate(moveit::planning_interface::MoveGroupInterface &move_group,
                        const geometry_msgs::Pose &target_pose );


#endif //SRC_MOVE_AUBO_API_H
