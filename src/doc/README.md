# szar_robot

#### 介绍
东北大学教育机器人开发平台

#### 软件架构
软件架构说明


#### 安装教程

1. 小车上linux系统配置已经预配置完成，不需要处理。
2. 控制端操作的电脑，需要运用Linux系统电脑。同时，需要安装ros。具体安装方式，请自行上网。
3. 控制端电脑安装成功ros之后，打开一个终端输入:
    终端：echo "export ROS_HOSTNAME=192.168.20.107  #your PC IP" >> ~/.bashrc 
    终端：echo "export ROS_MASTER_URI=http://192.168.20.101:11311   #robot IP, ros master" >> ~/.bashrc  
4. 关闭当前终端，之后就可以正常控制小车了。  

#### 使用说明

提示：
	除了运行rviz,其他所有的命令，都需要先输入：ssh pi@192.168.20.xxx 密码：pi
    每个工程任务相对独立，运行下一个任务时，需要关掉上一个工程的所有任务。
	
	
1. 数据测试：
    终端1：roslaunch smart_car smart_car_node.launch   #启动底盘通讯
    终端2：roslaunch data_calibration caliodom.launch std_Vx:=0.2 std_Lx:=1.0 Exl_name:=test Exl_address:=/home/pi/  #距离测量:速度Vx=0.2 m/s，距离Lx=1.0 m (或角度测量：std_Vw:=0.3 std_Lw:=90 ,转速:弧度每秒，角度单位:度)

    注： 重复测试时，终端1和终端2都需要同时关闭并开启，
         运行 数据测试 结束时，请关闭所有终端。

2. 建图操作：
    终端1：roslaunch bringup gmapping_bringup.launch  #启动机器，开启建图模式
    终端2：rosrun teleop_twist_keyboard teleop_twist_keyboard.py   #遥控辅助。注:遥控时，请保持点击该终端为活跃状态
    终端3：rviz   #显示地图信息等
    终端4：rosrun bringup auto_mapbuilding.sh  #自动建图指令，生成 auto_map 地图。也可自定义地图名字，运行下一步。
    终端4：rosrun bringup mapbuilding.sh  #手动处理地图，自定义命名，管理之前的地图。

    注： 运行 建图操作 结束时，请关闭所有终端。

3. 导航操作：
    终端1：roslaunch bringup navigation_bringup.launch map_name:=auto_map  #启动机器，开启导航模式。备注：后缀map_name:=auto_map 为地图选择，auto_map为自定义地图名字，可修改。此后缀也可省略，将默认加载auto_map地图。
    终端2：rosrun teleop_twist_keyboard teleop_twist_keyboard.py  #遥控辅助。注:遥控时，请保持点击该终端为活跃状态
    终端3：rviz  #显示地图信息等

    注： 运行 导航操作 结束时，请关闭所有终端。

备注：
    可能会用到：连接到服务器，sftp://192.168.20.135/home/pi 查看小车控制器源码文件。

    可能会遇到的一些问题:
    1. 运行 rosrun bringup auto_mapbuilding.sh 异常。 由于此指令为运行shell脚本，一般问题出现为没有给可执行权限。
        解决方法:roscd bringup/shell/ && chmod a+x ./* && cd
        之后，就可以了。

