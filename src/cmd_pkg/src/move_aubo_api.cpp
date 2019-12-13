//
// Created by wang on 12/6/19.
//

#include "move_aubo_api.h"

/*void print_aubo_state(moveit::planning_interface::MoveGroupInterface &move_group){
    // Get the coordinate system of the basic information

    //the Planning Frame , this is a constant
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // Get the end of the basic information
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Get the end pose (相对 /world)
    geometry_msgs::Pose current_pose;
    current_pose = move_group.getCurrentPose().pose;
    ROS_INFO_STREAM("current pose is "<<current_pose);
}*/

void move_by_joint(moveit::planning_interface::MoveGroupInterface &move_group,
                   const std::vector<double> &target_joint){

    move_group.setJointValueTarget(target_joint);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success=(move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

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

void move_Cartesian_path(moveit::planning_interface::MoveGroupInterface &move_group,
                         const geometry_msgs::Pose &start_wrist3_pose,
                         const geometry_msgs::Pose &target_wrist3_pose
        ){
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    //  Add three waypoints
    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(start_wrist3_pose);
    waypoints.push_back(target_wrist3_pose);

    // We want the Cartesian path to be interpolated at a resolution of 1 cm.
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;           //(The jump threshold is set to 0.0)
    const double eef_step = 0.01;                //(interpolation step)
    // Calculate Cartesian interpolation path: return path score (0~1, -1 stands for error)

    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if( (1 - fraction) < 0.001 )
    {
        ROS_INFO("move Cartesian path compute --- sucess");
        // 生成机械臂的运动规划数据
        plan.trajectory_ = trajectory;

        // 执行运动
        move_group.execute(plan);
        ROS_INFO("move Cartesian path --- sucess");
        sleep(1);
    }
    else
    {
        ROS_INFO("move Cartesian path compute --- faild with only %0.6f success ", fraction);
    }
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
    ROS_INFO("add desktop collision --- sucess");
}

void from_transform_to_pose(const tf::StampedTransform &transform, geometry_msgs::Pose &pose)
{
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z();
    pose.orientation.x = transform.getRotation().getX();
    pose.orientation.y = transform.getRotation().getY();
    pose.orientation.z = transform.getRotation().getZ();
    pose.orientation.w = transform.getRotation().getW();

}

void compute_wrist3_pose(const geometry_msgs::Pose &target_marker_pose ,
                         geometry_msgs::Pose &target_wrist3_pose,
                         const double &distance_gripper_w3,
                         const double &object_height ,
                         const bool &is_pickup){
    // base w3 gripper marker
    // 欧氏变换矩阵使用 Eigen::Isometry,齐次矩阵
    Eigen::Isometry3d T_w3_gripper = Eigen::Isometry3d::Identity();
    T_w3_gripper.pretranslate(Eigen::Vector3d ( 0,0, distance_gripper_w3));
    std::cout << "T_w3_gripper = \n" << T_w3_gripper.matrix() << std::endl;

    Eigen::Isometry3d T_base_marker = Eigen::Isometry3d::Identity();                // 虽然称为3d，实质上是4＊4的矩阵
    T_base_marker.rotate (Eigen::Quaterniond(target_marker_pose.orientation.w,
                                             target_marker_pose.orientation.x,
                                             target_marker_pose.orientation.y,
                                             target_marker_pose.orientation.z) );
    T_base_marker.pretranslate (Eigen::Vector3d (target_marker_pose.position.x,
                                                 target_marker_pose.position.y,
                                                 target_marker_pose.position.z) );
    std::cout << "T_base_marker = \n" << T_base_marker.matrix() << std::endl;


    Eigen::AngleAxisd rollAngle(3.1415926/2, Eigen::Vector3d::UnitX());
    Eigen::Isometry3d T_marker_gripper = Eigen::Isometry3d::Identity();

    if(is_pickup){// pick up
        T_marker_gripper.pretranslate(Eigen::Vector3d(0,0,-object_height/2));
    }
    else{//place
        T_marker_gripper.pretranslate(Eigen::Vector3d(0,0,object_height/2));
    }
    T_marker_gripper.rotate(rollAngle);
    std::cout << "T_marker_gripper = \n" << T_marker_gripper.matrix() << std::endl;

    Eigen::Isometry3d T_base_gripper( T_base_marker.matrix() * T_marker_gripper.matrix());
    std::cout << "T_base_gripper = \n" << T_base_gripper.matrix() << std::endl;


    Eigen::Isometry3d T_base_w3(T_base_gripper.matrix() * T_w3_gripper.matrix().inverse() ) ;
    Eigen::Matrix3d m_base_w3 = T_base_w3.matrix().block(0, 0, 3, 3);
    Eigen::Quaterniond q(m_base_w3);
    Eigen::Vector3d v_base_w3(T_base_w3.matrix().block(0, 3, 3, 1) );
    target_wrist3_pose.position.x = v_base_w3(0);
    target_wrist3_pose.position.y = v_base_w3(1);
    target_wrist3_pose.position.z = v_base_w3(2);
    target_wrist3_pose.orientation.x = q.x();
    target_wrist3_pose.orientation.y = q.y();
    target_wrist3_pose.orientation.z = q.z();
    target_wrist3_pose.orientation.w = q.w();

/*    // broadcaseter the tf between base_link  and gripper_link
    tf::TransformBroadcaster br;
    tf::Transform transform;
    Eigen::Vector3d v_base_gripper(T_base_w3.matrix().block(0,3,3,1) );
    transform.setOrigin(tf::Vector3( v_base_gripper(0) , v_base_gripper(1) , v_base_gripper(2) ));
    transform.setRotation(tf::Quaternion(q.x() , q.y() , q.z() , q.w())); // w3 and gripper have the same q
    std::cout<<"发布tf变换：sendTransform函数"<<std::endl;
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"/base_link","/gripper_link"));*/
}

void get_marker_pose(const std::string marker_frame , geometry_msgs::Pose &target_marker_pose){
    //******look up the tranform between usb_rgb_optical_frame and camera_marker
    tf::TransformListener listener;
    //1. 阻塞直到frame相通
    std::cout<<"1. 阻塞直到frame相通"<<std::endl;
    listener.waitForTransform("/base_link", marker_frame, ros::Time(0), ros::Duration(10.0));
    if(!listener.canTransform("/base_link", marker_frame, ros::Time(0) ) ){
        ROS_ERROR("%s","waitForTransform timeout");
    }
    tf::StampedTransform transform_marker;
    //2. 监听对应的tf,返回平移和旋转
    std::cout<<"2. 监听对应的tf,返回平移和旋转"<<std::endl;
    listener.lookupTransform("/base_link", marker_frame,
                             ros::Time(0), transform_marker);        //ros::Time(0)表示最近的一帧坐标变换
    from_transform_to_pose(transform_marker , target_marker_pose);
    std::cout<<"输出的位置坐标：x="<<transform_marker.getOrigin().x()<<",y="<<transform_marker.getOrigin().y()<<",z="<<transform_marker.getOrigin().z()<<std::endl;
    std::cout<<"输出的旋转四元数：w="<<transform_marker.getRotation().getW()<<",x="<<transform_marker.getRotation().getX()<<
             ",y="<<transform_marker.getRotation().getY()<<",z="<<transform_marker.getRotation().getZ()<<std::endl;
    ROS_INFO_STREAM("marker object pose is " << target_marker_pose);
}

