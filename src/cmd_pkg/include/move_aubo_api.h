//
// Created by wang on 12/6/19.
//

#ifndef SRC_MOVE_AUBO_API_H
#define SRC_MOVE_AUBO_API_H



#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_listener.h>


#include <Eigen/Core>
// Eigen 几何模块
#include <Eigen/Geometry>
void print_aubo_state(moveit::planning_interface::MoveGroupInterface &move_group);

void move_by_joint(moveit::planning_interface::MoveGroupInterface &move_group,
                   const std::vector<double> &target_joint);

void move_by_coordinate(moveit::planning_interface::MoveGroupInterface &move_group,
                        const geometry_msgs::Pose &target_pose,
                        const std::string& end_effector_link = "");

void move_with_orientationConstraint(moveit::planning_interface::MoveGroupInterface &move_group,
                        const geometry_msgs::Pose &start_pose,
                        const geometry_msgs::Pose &target_pose,
                        const std::string& end_effector_link = "wrist3_Link");
void move_Cartesian_path(moveit::planning_interface::MoveGroupInterface &move_group,
                         const geometry_msgs::Pose &start_wrist3_pose,
                         const geometry_msgs::Pose &target_wrist3_pose
);
void add_desktop_collision(moveit::planning_interface::MoveGroupInterface &move_group,
                           moveit::planning_interface::PlanningSceneInterface &planning_scene_interface);

void from_transform_to_pose(const tf::StampedTransform &transform,geometry_msgs::Pose &pose);

void compute_wrist3_pose(const geometry_msgs::Pose &target_marker_pose ,
                        geometry_msgs::Pose &target_wrist3_pose,
                         const double &distance_gripper_w3,
                         const double &object_height,
                         const bool &is_pickup);
void get_marker_pose(const std::string marker_frame , geometry_msgs::Pose &target_marker_pose);
#endif //SRC_MOVE_AUBO_API_H
