# mobile_robot_exp
Mobile Robot Experiment in 2019 Autumn

## 1.实验结果

实现基于marker码识别的aubo i5机械臂码垛实验

## 2. 环境配置

参考：

 https://blog.csdn.net/harrycomeon/article/details/102600957

`与aubo通信`:https://blog.csdn.net/cstone123/article/details/95608424

`aruco_ros`:https://github.com/lg609/aubo_robot.git

`vision_visp`:https://github.com/lagadic/vision_visp.git

`easy_handeye`:https://github.com/IFL-CAMP/easy_handeye

`usb_cam`:https://github.com/bosch-ros-pkg/usb_cam.git  

## 3文件说明

`cmd_pkg`

├── CMakeLists.txt
├── config
│   ├── easy_handeye_eye_on_hand.yaml
│   └── map
├── include
│   └── move_all_base.h
├── launch
│   └── aubo_bringup.launch
├── package.xml
└── src
    ├── aubo_add_collision.cpp
    ├── exp4_1.py
    ├── exp4_2.py
    ├── get_pose.cpp
    ├── move_all_base.cpp
    └── move_all.cpp

`doc`
├── commands.md
└── README.md

```
move_all.cpp是主程序
aubo_add_collision.cpp是添加模拟的夹爪
exp4_1.py和exp4_2.py是控制底盘以两种方式运动画圆的程序
```