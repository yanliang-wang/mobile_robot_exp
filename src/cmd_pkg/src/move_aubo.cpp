#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "move_aubo_api.h"

int main(int argc,char** argv)
{
    //initialize
    ros::init(argc, argv, "move_aubo");
    ros::NodeHandle n;
    ros::AsyncSpinner spin(1);
    spin.start();

    moveit::planning_interface::MoveGroupInterface move_group("manipulator_i5");
    std::string reference_frame="base_link";
    move_group.setPoseReferenceFrame(reference_frame);//reference frame


    //output some info
    print_aubo_state(move_group);

/*    move_group.allowReplanning(true);
    move_group.setMaxVelocityScalingFactor(0.4);
    move_group.setMaxAccelerationScalingFactor(0.3);
    move_group.setGoalOrientationTolerance(0.01);
    move_group.setGoalPositionTolerance(0.01);*/
    // define the home position
    std::vector<double> home_position;
    home_position.push_back(-0.001255);
    home_position.push_back(-0.148822);
    home_position.push_back(-1.406503);
    home_position.push_back(0.311441);
    home_position.push_back(-1.571295);
    home_position.push_back(-0.002450);

    move_by_joint(move_group,home_position);

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
    move_by_coordinate(move_group,target_pose);

    ros::waitForShutdown();
    return 0;
}


