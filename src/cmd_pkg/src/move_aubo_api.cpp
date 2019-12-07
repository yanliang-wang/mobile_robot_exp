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
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success=(move_group.plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success){
        move_group.move();
        ROS_INFO("move by joint --- sucess");
    }
    else{
        ROS_INFO("move by joint --- failed");
    }
}

void move_by_coordinate(moveit::planning_interface::MoveGroupInterface &move_group,
                        const geometry_msgs::Pose &target_pose, 
                        const std::string& end_effector_link ){
    // planning and moving to a target pose
    // the target_pose is in reference frame
    move_group.setPoseTarget(target_pose,end_effector_link);//"" 第二个参数 指定末端执行器的link ，默认为group中的最后一个link

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success){
        move_group.execute(my_plan);
        ROS_INFO("move by coordinate --- sucess");
    }
    else{
        ROS_INFO("move by coordinate --- failed");
    }


}

void move_with_orientationConstraint(moveit::planning_interface::MoveGroupInterface &move_group,
                                   const geometry_msgs::Pose &start_pose,
                                   const geometry_msgs::Pose &target_pose,
                                   const std::string& end_effector_link){
    const double position_tolerance = 0.2;

    //inspect the invalid values
    if( (fabs(start_pose.position.x - target_pose.position.x) > position_tolerance)  || \
        (fabs(start_pose.position.y - target_pose.position.y) > position_tolerance) || \
        (fabs(start_pose.position.z - target_pose.position.z) > position_tolerance) ){
        ROS_INFO("move with orientation constraint --- can't meet the condition");
        return;
    }
    //确保机械臂状态处于起始状态
    move_group.setPoseTarget(start_pose);
    move_group.move();
    ROS_INFO("move with orientation constraint --- initial pose is valid");
    // Set the pose to be constrained at the end (consistent with base_link according to the settings)
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = end_effector_link;
    ocm.header.frame_id = move_group.getPoseReferenceFrame();

    ocm.orientation.w = start_pose.orientation.w;
    ocm.orientation.x = start_pose.orientation.x;
    ocm.orientation.y = start_pose.orientation.y;
    ocm.orientation.z = start_pose.orientation.z;
    ocm.absolute_x_axis_tolerance = position_tolerance;   //(Specify the tolerance of the axis)
    ocm.absolute_y_axis_tolerance = position_tolerance;
    ocm.absolute_z_axis_tolerance = position_tolerance;
    ocm.weight = 1.0;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // Add path constraints
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(test_constraints);




    //planning the motion
    move_group.setPoseTarget(target_pose);

    move_group.setPlanningTime(20.0);
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success){
        move_group.execute(my_plan);
        ROS_INFO("move with orientation constraint --- sucess");
    }
    else{
        ROS_INFO("move with orientation constraint --- failed");
    }

    // clear path constraint
    move_group.clearPathConstraints();
}

void add_desktop_collision(moveit::planning_interface::MoveGroupInterface &move_group,
                           moveit::planning_interface::PlanningSceneInterface &planning_scene_interface){

    //******* Defining a new desktop added in the world
    moveit_msgs::CollisionObject static_object;
    static_object.header.frame_id = move_group.getPlanningFrame();
    // Set the ID of the object
    static_object.id = "desktop";


    // Set the info of the basic desktop (length, width and height, pose)
    shape_msgs::SolidPrimitive primitive_basic;
    primitive_basic.type = primitive_basic.BOX;
    primitive_basic.dimensions.resize(3);
    primitive_basic.dimensions[0] = 2.5;
    primitive_basic.dimensions[1] = 2.5;
    primitive_basic.dimensions[2] = 0.05;

    geometry_msgs::Pose basic_pose;
    basic_pose.orientation.w = 1.0;
    basic_pose.position.x = 0.0;
    basic_pose.position.y = 0.0;
    basic_pose.position.z = 0.5 - primitive_basic.dimensions[2] / 2;

    // Set the info of the target desktop (length, width and height, pose)
/*    shape_msgs::SolidPrimitive primitive_target;
    primitive_target.type = primitive_target.BOX;
    primitive_target.dimensions.resize(3);
    primitive_target.dimensions[0] = 1.7;
    primitive_target.dimensions[1] = 1.7;
    primitive_target.dimensions[2] = 0.05;

    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.0;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0;*/

    static_object.primitives.push_back(primitive_basic);
    //static_object.primitives.push_back(primitive_target);
    static_object.primitive_poses.push_back(basic_pose);
    //static_object.primitive_poses.push_back(target_pose);
    static_object.operation = static_object.ADD;

    //add the collision objects to the planning scene
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(static_object);
    planning_scene_interface.addCollisionObjects(collision_objects);

}
