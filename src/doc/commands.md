# 移动机器人实验说明
## 1. 底座相关
### (1) 与底座通信

在上位机的.bashrc中修改：

```
export ROS_HOSTNAME=192.168.20.114
export ROS_MASTER_URI=http://192.168.20.103:11311
```
打开终端执行下面指令：

`底盘终端：roslaunch smart_car smart_car_node.launch  #启动底盘通讯`

对底盘进行底盘控制：

`底盘终端：rosrun teleop_twist_keyboard teleop_twist_keyboard.py #遥控辅助 `

### (2)建图

```
上位机终端：roslaunch bringup gmapping_bringup.launch  #启动机器，开启建图模式
上位机终端：rosrun teleop_twist_keyboard teleop_twist_keyboard.py   #遥控辅助
上位机终端：rviz   #然后打开gmapping.rviz

上位机终端：rosrun map_server map_saver -f ~/robot1_map    #建图完成保存地图
```

## 2. 机械臂相关

以下所有指令均在上位机终端执行。

### (1) 机械臂通信

#### a. 机械臂仿真

在rviz上进行仿真

`roslaunch aubo_i5_moveit_config moveit_planning_execution.launch robot_ip:=127.0.0.1`

在gazebo上进行仿真

`roslaunch aubo_gazebo aubo_i5_gazebo_control.launch`

#### b.机械臂真机通信

`roslaunch aubo_i5_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.20.106`

### (2)手眼标定

```
roslaunch usb_cam usb_cam-test.launch  #打开摄像头，需要在里面设置video_device的值，即相机的设备号
roslaunch easy_handeye aubo_calibration.launch #启动aruco_tracker节点，并与机械臂通信
roslaunch easy_handeye easy.launch  #进行眼在手上的标定
```

### (3)程序节点

`move_aubo`一些机械臂的运动测试实验

`aubo_add_collision`添加机械臂的碰撞物

## 3. 注意事项

### (1) 控制机械臂运动

在orientationConstraint测试中，会出现Controller is taking too long to execute trajectory的错误信息，在aubo_robot-master/aubo_i5_moveit_config/launch/trajectory_execution.launch.xml中修改下面的第一行，并添加第二行可以避免这个错误信息

```
  <param name="trajectory_execution/allowed_execution_duration_scaling" value="4.0"/> <!-- default 1.2 -->
  <param name="trajectory_execution/execution_duration_monitoring" value="false" />
```
### (2)  simulatin test

`roslaunch cmd_pkg aubo_bringup.launch`

`rosrun cmd_pkg aubo_add_collision`

`rosrun cmd_pkg move_all`

`rostopic echo /move_base/result`   `rostopic echo /hand_control_cmd`

```
rostopic pub /move_base_simple/goal geometry_msgs/seStamped "header:
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
```

### (3)真机Debug

#### a. 测试gripper

#### b. 手眼标定，看多张aruco码的识别效果，看根据marker得到的gripper位姿是否合理

#### c. 找一个效果好的实验室地图，测试 底盘导航

#### d. 准备实验物品

比底盘稍微高一点的凳子

贴有aruco码的长方体物体，和一个相同尺寸的aruco码 