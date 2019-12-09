//
// Created by wang on 12/8/19.
//

#ifndef SRC_MOVE_BASIC_API_H
#define SRC_MOVE_BASIC_API_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <tf/transform_listener.h>
//movieit relative head file
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
// gripper relative head file
#include <hand_control/hand_control_cmd.h>
#define PI 3.1415926

struct MOVE_ALL : public moveit::planning_interface::MoveGroupInterface{

    ros::NodeHandle n;
    ros::Publisher pub_basic_point , pub_gripper_command;
    ros::Subscriber sub ;

    // some variables
    bool sim;
    // some constants
    const double height_world_base = 1.5105 - 1.0085;
    std::vector<double> home_joint;
    hand_control::hand_control_cmd msg_open , msg_close;
    const std::string reference_frame="base_link";
    const std::string end_effector_link = "wrist3_Link";
    std::string marker_frame ;//camera_marker, set /wrist3_Link to simulate
    const double distance_gripper_w3 = 0.15;// the distance between gripper_link and wrist3_Link

    //define a planning scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;// Create a planning scene interface object

    MOVE_ALL();
    void movebaseResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg);
    void print_aubo_state();
    void add_desktop_collision();
    void move_by_joint(const std::vector<double> &target_joint);
    void move_Cartesian_path(const geometry_msgs::Pose &start_wrist3_pose,
                             const geometry_msgs::Pose &target_wrist3_pose);
    void get_marker_pose(const std::string &marker_frame , geometry_msgs::Pose &target_marker_pose);
    void compute_wrist3_pose(const geometry_msgs::Pose &target_marker_pose ,
                             geometry_msgs::Pose &target_wrist3_pose,
                             const double &distance_gripper_w3,
                             const double &object_height ,
                             const bool &is_pickup);
    void from_transform_to_pose(const tf::StampedTransform &transform,geometry_msgs::Pose &pose);

};


#endif //SRC_MOVE_BASIC_API_H
