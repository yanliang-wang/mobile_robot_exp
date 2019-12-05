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

`roslaunch moveit_planning_execution.launch robot_ip:=127.0.0.1`

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

