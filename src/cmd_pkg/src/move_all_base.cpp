//
// Created by wang on 12/8/19.
//
#include "move_all_base.h"

MOVE_ALL::MOVE_ALL(): moveit::planning_interface::MoveGroupInterface("manipulator_i5")
{

    //************ define publisher and subscriber
    pub_basic_point = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 2);;
    pub_gripper_command = n.advertise<hand_control::hand_control_cmd>("hand_control_cmd", 1000);
    //订阅导航结果
    sub = n.subscribe("/move_base/result" , 1 , &MOVE_ALL::movebaseResultCallback,this);
    setPoseReferenceFrame(reference_frame);//reference frame
    //设置允许的最大速度和加速度
    setMaxAccelerationScalingFactor(0.2);
    setMaxVelocityScalingFactor(0.2);
    //当运动规划失败后，允许重新规划
    allowReplanning(true);

    //initialize hand msg and marker frame
    msg_open.msgtype = "Open";
    msg_open.value = 1;
    msg_close.msgtype = "Close";
    msg_close.value = 1;
    msg_enable.msgtype = "Enble";
    msg_enable.value = 1;
    marker_frame = "/marker_frame";

    //output some basic moveit information
    print_aubo_state();
    //add the basic desktop collision object
    add_desktop_collision();
    //define a home state and move to it
/*    home_joint.push_back(-1.11018);
    home_joint.push_back(-0.148822);
    home_joint.push_back(-1.406503);
    home_joint.push_back(0.311441);
    home_joint.push_back(-1.571295);
    home_joint.push_back(-0.002450);*/
    home_joint.push_back(-1.11018);
    home_joint.push_back(0.146884);
    home_joint.push_back(-1.13574);
    home_joint.push_back(0.286447);
    home_joint.push_back(-1.57164);
    home_joint.push_back(0.554514);
    //define a home state and move to it
    home_joint_grip.push_back(-1.11018);
    home_joint_grip.push_back(0.146884);
    home_joint_grip.push_back(-1.13574);
    home_joint_grip.push_back(0.286447);
    home_joint_grip.push_back(-1.57164);
    home_joint_grip.push_back(0.554514);
/*    home_joint_grip.push_back(-1.11954);
    home_joint_grip.push_back(0.509547);
    home_joint_grip.push_back(-0.638324);
    home_joint_grip.push_back(0.421445);
    home_joint_grip.push_back(-1.57202);
    home_joint_grip.push_back(0.795159);*/
    move_by_joint( home_joint);
    geometry_msgs::Pose initial_marker_pose;
    initial_marker_pose;
    get_marker_pose(marker_frame, initial_marker_pose);
    pub_gripper_command.publish(msg_enable);
    pub_gripper_command.publish(msg_close);
}

void MOVE_ALL::movebaseResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg)
{
    /*
rostopic pub /move_base_simple/goal geomey_msgs/PoseStamped "header:
seq: 0
stamp:
secs: 0
nsecs: 0
frame_id: 'map'
pose:
position:
x: -0.51489508152
y: -0.530243039131
z: 0.0
orientation:
x: 0.0
y: 0.0
z: -0.705563271538
w: 0.708646928912"
 * */
    sleep(1.0);
    static int num = 0;
    ++num;
    ROS_INFO("The goal %d is reached!", num );

    if(num == 1 )
    {
        //**************first point --- pick up
        ROS_INFO("Open the gripper!");
        pub_gripper_command.publish(msg_enable);
        sleep(0.5);
        pub_gripper_command.publish(msg_open);//open the gripper
        move_by_joint(home_joint_grip); //move to the pose to grip
        //****** get the marker pose
        geometry_msgs::Pose target_marker_pose;
        get_marker_pose(marker_frame,target_marker_pose);
        //****** compute the pose to grip
        geometry_msgs::Pose target_wrist3_pose;
        compute_wrist3_pose(target_marker_pose,target_wrist3_pose , distance_gripper_w3 , object_height, true);
        //****** execute the grip
        geometry_msgs::Pose currnt_wrist3_pose = getCurrentPose().pose;//relative to world
        currnt_wrist3_pose.position.z -= height_world_base;
        target_wrist3_pose.orientation = currnt_wrist3_pose.orientation;
        geometry_msgs::Pose start_wrist3_pose = target_wrist3_pose;
        start_wrist3_pose.position.z = currnt_wrist3_pose.position.z ;//relative to world
        move_by_coordinate(start_wrist3_pose);
        move_by_coordinate(target_wrist3_pose);
//        move_Cartesian_path( start_wrist3_pose, target_wrist3_pose);
        sleep(1.0);
        //****** grip and make the aubo move to home state
        ROS_INFO("Close the gripper!");
        pub_gripper_command.publish(msg_enable);
        sleep(0.5);
        pub_gripper_command.publish(msg_close);//close the gripper
        sleep(1.0);
//        move_Cartesian_path( target_wrist3_pose,start_wrist3_pose);
        move_by_coordinate(start_wrist3_pose);
        move_by_joint(home_joint);

        //****** move to the position to place the object
        geometry_msgs::PoseStamped basic_home_target;
        //basic_target.header.seq = 1;
        basic_home_target.header.stamp = ros::Time::now();
        basic_home_target.header.frame_id = "map";
        basic_home_target.pose.position.x = -0.555596232414;
        basic_home_target.pose.position.y = -1.29687714577;
        basic_home_target.pose.position.z = 0.0;
        basic_home_target.pose.orientation.x = 0.0;
        basic_home_target.pose.orientation.y = 0.0;
        basic_home_target.pose.orientation.z = -0.71561187448;
        basic_home_target.pose.orientation.w = 0.698498135361;
        pub_basic_point.publish(basic_home_target);

    }
    if(num == 1 ){
        //************* home point --- place
        move_by_joint(home_joint_grip);//move to the pose to place

        geometry_msgs::Pose target_marker_pose;
        get_marker_pose(marker_frame,target_marker_pose);
        //****** compute the pose to place
        geometry_msgs::Pose target_wrist3_pose;
        compute_wrist3_pose(target_marker_pose,target_wrist3_pose , distance_gripper_w3 , object_height, false);
        target_wrist3_pose.position.z += object_height;
        //****** execute the grip
        geometry_msgs::Pose currnt_wrist3_pose = getCurrentPose().pose;//relative to world
        target_wrist3_pose.orientation = currnt_wrist3_pose.orientation;
        geometry_msgs::Pose start_wrist3_pose = target_wrist3_pose;
        currnt_wrist3_pose.position.z -= height_world_base;
        start_wrist3_pose.position.z = currnt_wrist3_pose.position.z ;//relative to world
        move_by_coordinate(start_wrist3_pose);
        move_by_coordinate(target_wrist3_pose);
//        move_Cartesian_path( start_wrist3_pose, target_wrist3_pose);
        sleep(1.0);
        //****** grip and make the aubo move to home state
        ROS_INFO("Open the gripper!");
        pub_gripper_command.publish(msg_enable);
        sleep(0.5);
        pub_gripper_command.publish(msg_open);//open the gripper
        sleep(1.0);
//        move_Cartesian_path( target_wrist3_pose , start_wrist3_pose);
        move_by_coordinate(start_wrist3_pose);
        move_by_joint(home_joint);

        //****** move to the other position
        geometry_msgs::PoseStamped basic_other_target;
        //basic_other_target.header.seq = 1;
        basic_other_target.header.stamp = ros::Time::now();
        basic_other_target.header.frame_id = "map";
        basic_other_target.pose.position.x = -0;
        basic_other_target.pose.position.y = -1;
        basic_other_target.pose.position.z = 0.0;
        basic_other_target.pose.orientation.x = 0.0;
        basic_other_target.pose.orientation.y = 0.0;
        basic_other_target.pose.orientation.z = 0.0;
        basic_other_target.pose.orientation.w = 1;
        pub_basic_point.publish(basic_other_target);
    }
}

void MOVE_ALL::print_aubo_state()
{
    // Get the coordinate system of the basic information

    //the Planning Frame , this is a constant
    ROS_INFO("Planning frame: %s", getPlanningFrame().c_str());
    // Get the end of the basic information
    ROS_INFO("End effector link: %s", getEndEffectorLink().c_str());
    // Get the end pose (相对 /world)

    geometry_msgs::Pose current_pose;
    current_pose = getCurrentPose().pose;
    ROS_INFO_STREAM("current pose is "<<current_pose);
}

void MOVE_ALL::add_desktop_collision()
{
    //******* Defining a new desktop added in the world
    moveit_msgs::CollisionObject static_object;
    static_object.header.frame_id = getPlanningFrame();
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

void MOVE_ALL::move_by_joint(const std::vector<double> &target_joint)
{
    setJointValueTarget(target_joint);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success=(plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if(success){
        move();
        ROS_INFO("move by joint --- sucess");
    }
    else{
        ROS_INFO("move by joint --- failed");
    }
}

void MOVE_ALL::move_Cartesian_path(const geometry_msgs::Pose &start_wrist3_pose,
                                   const geometry_msgs::Pose &target_wrist3_pose)
{
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

    double fraction = computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if( (1 - fraction) < 0.001 )
    {
        ROS_INFO("move Cartesian path compute --- sucess");
        // 生成机械臂的运动规划数据
        plan.trajectory_ = trajectory;

        // 执行运动
        execute(plan);
        ROS_INFO("move Cartesian path --- sucess");
        sleep(1);
    }
    else
    {
        ROS_INFO("move Cartesian path compute --- faild with only %0.6f success ", fraction);
    }
}

void MOVE_ALL::get_marker_pose(const std::string &marker_frame, geometry_msgs::Pose &target_marker_pose)
{
    //******look up the tranform between usb_rgb_optical_frame and camera_marker
    //1. 阻塞直到frame相通
    std::cout<<"1. 阻塞直到frame相通"<<std::endl;
    listener.waitForTransform("/base_link",marker_frame,  ros::Time(), ros::Duration(10.0));
    if(!listener.canTransform("/base_link",marker_frame,  ros::Time() ) ){
        ROS_ERROR("%s","waitForTransform timeout");
    }
    tf::StampedTransform transform_marker;
    //2. 监听对应的tf,返回平移和旋转
    std::cout<<"2. 监听对应的tf,返回平移和旋转"<<std::endl;
    listener.lookupTransform("/base_link",marker_frame,
                             ros::Time(0), transform_marker);        //ros::Time(0)表示最近的一帧坐标变换
    tf::Quaternion q = transform_marker.getRotation();
    tf::Vector3 v = transform_marker.getOrigin();

    from_transform_to_pose(transform_marker , target_marker_pose);
/*    std::cout<<"输出的位置坐标：x="<<transform_marker.getOrigin().x()<<",y="<<transform_marker.getOrigin().y()<<",z="<<transform_marker.getOrigin().z()<<std::endl;
    std::cout<<"输出的旋转四元数：w="<<transform_marker.getRotation().getW()<<",x="<<transform_marker.getRotation().getX()<<
             ",y="<<transform_marker.getRotation().getY()<<",z="<<transform_marker.getRotation().getZ()<<std::endl;*/
    std::cout<<"输出的位置坐标：x="<<v.getX()<<",y="<<v.getY()<<",z="<<v.getZ()<<std::endl;
    std::cout<<"输出的旋转四元数：w="<<q.getW()<<",x="<<q.getX()<<
             ",y="<<q.getY()<<",z="<<q.getZ()<<std::endl;
    ROS_INFO_STREAM("marker object pose is " << target_marker_pose);
}

void
MOVE_ALL::compute_wrist3_pose(const geometry_msgs::Pose &target_marker_pose, geometry_msgs::Pose &target_wrist3_pose,
                              const double &distance_gripper_w3, const double &object_height, const bool &is_pickup)
{
    // base w3 gripper marker
    // 欧氏变换矩阵使用 Eigen::Isometry,齐次矩阵
    Eigen::Isometry3d T_w3_gripper = Eigen::Isometry3d::Identity();
    T_w3_gripper.pretranslate(Eigen::Vector3d ( 0,0, distance_gripper_w3));
    std::cout << "T_w3_gripper = \n" << T_w3_gripper.matrix() << std::endl;

    Eigen::Isometry3d T_base_marker = Eigen::Isometry3d::Identity();                // 虽然称为3d，实质上是4＊4的矩阵
    T_base_marker.translate (Eigen::Vector3d (target_marker_pose.position.x,
                                              target_marker_pose.position.y,
                                              target_marker_pose.position.z) );
    T_base_marker.rotate (Eigen::Quaterniond(target_marker_pose.orientation.w,
                                             target_marker_pose.orientation.x,
                                             target_marker_pose.orientation.y,
                                             target_marker_pose.orientation.z) );//right multiply

    std::cout << "T_base_marker = \n" << T_base_marker.matrix() << std::endl;


    Eigen::AngleAxisd rollAngle(3.1415926/2, Eigen::Vector3d::UnitX());
    Eigen::Isometry3d T_marker_gripper = Eigen::Isometry3d::Identity();

    if(is_pickup){// pick up
        T_marker_gripper.pretranslate(Eigen::Vector3d(0,0,0));
    }
    else{//place
        T_marker_gripper.pretranslate(Eigen::Vector3d(0,0,0));
    }
    T_marker_gripper.rotate(rollAngle);
    std::cout << "T_marker_gripper = \n" << T_marker_gripper.matrix() << std::endl;

    Eigen::Isometry3d T_base_gripper( T_base_marker.matrix() * T_marker_gripper.matrix());
    std::cout << "T_base_gripper = \n" << T_base_gripper.matrix() << std::endl;


    Eigen::Isometry3d T_base_w3( T_base_gripper.matrix() * T_w3_gripper.matrix().inverse() ) ;
    std::cout << "T_base_w3 = \n" << T_base_w3.matrix() << std::endl;
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

}

void MOVE_ALL::from_transform_to_pose(const tf::StampedTransform &transform, geometry_msgs::Pose &pose)
{
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z();
    pose.orientation.x = transform.getRotation().getX();
    pose.orientation.y = transform.getRotation().getY();
    pose.orientation.z = transform.getRotation().getZ();
    pose.orientation.w = transform.getRotation().getW();
}

void MOVE_ALL::move_by_coordinate(const geometry_msgs::Pose &target_pose)
{
    // planning and moving to a target pose
    // the target_pose is in reference frame
    setPoseTarget(target_pose);//"" 第二个参数 指定末端执行器的link ，默认为group中的最后一个link

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success){
        execute(my_plan);
        ROS_INFO("move by coordinate --- sucess");
    }
    else{
        ROS_INFO("move by coordinate --- failed");
    }
}



