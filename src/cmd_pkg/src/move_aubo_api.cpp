//
// Created by wang on 12/6/19.
//

#include "move_aubo_api.h"

void print_aubo_state(moveit::planning_interface::MoveGroupInterface &move_group){
    // Get the coordinate system of the basic information

    //the Planning Frame , this is a constant
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // Get the end of the basic information
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Get the end pose (相对 /world)
    geometry_msgs::Pose current_pose;
    current_pose = move_group.getCurrentPose().pose;
    ROS_INFO_STREAM("current pose is "<<current_pose);
}

void move_by_joint(moveit::planning_interface::MoveGroupInterface &move_group,
                   const std::vector<double> &target_joint){

    move_group.setJointValueTarget(target_joint);
    move_group.move();
}

void move_by_coordinate(moveit::planning_interface::MoveGroupInterface &move_group,
                        const geometry_msgs::Pose &target_pose){
    // planning and moving to a target pose
    // the target_pose is in reference frame
    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO(" pose goal %s", success ? "Success" : "FAILED");
    move_group.execute(my_plan);
}