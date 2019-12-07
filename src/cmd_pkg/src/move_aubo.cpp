#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/LinearMath/Quaternion.h>

#include "move_aubo_api.h"
#define PI 3.1415926

int main(int argc, char** argv)
{
    //initialize
    ros::init(argc, argv, "move_aubo");
    ros::NodeHandle n;
    ros::AsyncSpinner spin(1);
    spin.start();

    moveit::planning_interface::MoveGroupInterface move_group("manipulator_i5");
    const std::string reference_frame="base_link";
    const std::string end_effector_link = "wrist3_Link";
    const double gripper_lengeth = 0.15;
    move_group.setPoseReferenceFrame(reference_frame);//reference frame
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;// Create a planning scene interface object

    //output some basic information
    print_aubo_state(move_group);
    //add the basic desktop collision object
    add_desktop_collision(move_group, planning_scene_interface);
    //************  some motion experiments


    // define the home position
    std::vector<double> home_joint;
    home_joint.push_back(-0.001255);
    home_joint.push_back(-0.148822);
    home_joint.push_back(-1.406503);
    home_joint.push_back(0.311441);
    home_joint.push_back(-1.571295);
    home_joint.push_back(-0.002450);

    move_by_joint(move_group, home_joint);

    // Set the target pose , RPY mode (rotation around the reference axis X, Y, Z)
    geometry_msgs::Pose target_pose;
    tf::Quaternion q;
    q.setRPY(3.14,0,-1.57);       //radian
    target_pose.position.x = -0.4;
    target_pose.position.y = -0.3;
    target_pose.position.z = 0.30;
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();
    //move_by_coordinate(move_group,target_pose);

    // rotate around a joint
    std::vector<double> target_joint;
    target_joint = move_group.getCurrentJointValues();
    target_joint[0] = - 1.57;
    //move_by_joint(move_group, target_joint);

    //orientation constraint test
    geometry_msgs::Pose     start_pose2;
    start_pose2.position.x = -0.4;
    start_pose2.position.y = 0.05;
    start_pose2.position.z = 0.54;
    start_pose2.orientation.x = q.x();
    start_pose2.orientation.y = q.y();
    start_pose2.orientation.z = q.z();
    start_pose2.orientation.w = q.w();

    geometry_msgs::Pose target_pose3_1;
    target_pose3_1.position.x = -0.4;
    target_pose3_1.position.y = -0.1;
    target_pose3_1.position.z = 0.41;
    target_pose3_1.orientation.x = q.x();
    target_pose3_1.orientation.y = q.y();
    target_pose3_1.orientation.z = q.z();
    target_pose3_1.orientation.w = q.w();

    move_group.setPoseTarget(target_pose3_1);
    move_with_orientationConstraint(move_group,start_pose2,target_pose3_1);

    move_by_joint(move_group,home_joint);

    ros::shutdown();
    return 0;
}


