//************* add the collision of gripper and attach it to the wrist3_Link
//by wang
//
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf/tf.h>
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

    sleep(3.0);
    //****** Define a collision object
    moveit_msgs::CollisionObject dynamic_object;
    dynamic_object.header.frame_id = move_group.getPlanningFrame();
    //dynamic_object.header.frame_id = end_effector_link;
    // Set the ID of the object
    dynamic_object.id = "gripper";
    //Defining a box (gripper) added to the world
    shape_msgs::SolidPrimitive primitive_gripper;
    primitive_gripper.type = primitive_gripper.CYLINDER;
    primitive_gripper.dimensions.resize(3);
    primitive_gripper.dimensions[0] = gripper_lengeth;// the length of the gripper
    primitive_gripper.dimensions[1] = 0.035;//radius
    // Set the gripper pose
    geometry_msgs::Pose gripper_pose;
    tf::Quaternion gripper_orientation;
    gripper_orientation.setRPY(PI/2,0,0);       //radian
    gripper_pose.orientation.x = gripper_orientation.x();
    gripper_pose.orientation.y = gripper_orientation.y();
    gripper_pose.orientation.z = gripper_orientation.z();
    gripper_pose.orientation.w = gripper_orientation.w();
    gripper_pose.position.x = 0;
    gripper_pose.position.y = -0.215501 - primitive_gripper.dimensions[0]/2 - 0.005;
    gripper_pose.position.z = 1.5105;
    dynamic_object.primitives.push_back(primitive_gripper);
    dynamic_object.primitive_poses.push_back(gripper_pose);
    dynamic_object.operation = dynamic_object.ADD;

    //****** add the collision objects to the planning scene
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(dynamic_object);
    planning_scene_interface.addCollisionObjects(collision_objects);

    move_group.attachObject(dynamic_object.id);

    ros::shutdown();
    return 0;
}


