#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
from geometry_msgs.msg import Twist
from math import pi
from math import cos
from math import sin
"""
逆时针（顺时针） 角速度为0.25rad/s 半径为1m旋转 
"""

class OutAndBack():
    def __init__(self):
        # Give the node a name
        rospy.init_node('out_and_back', anonymous=False)
        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # How fast will we update the robot's movement?
        rate = 50
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)
        

        #angular_speed = 0.25#逆时针为0.25 顺时针为-0.25
        radius = 1
        angular_speed = 0.25
        linear_speed = angular_speed * radius

        move_cmd = Twist()
        circles_count = 1 #圈数
        duration = circles_count * 2 * pi / angular_speed
        ticks = int(duration * rate)


        for t in range(ticks):
            move_cmd.linear.x = linear_speed
            move_cmd.linear.y = 0
            move_cmd.linear.z = 0
            move_cmd.angular.x = 0
            move_cmd.angular.y = 0
            move_cmd.angular.z = angular_speed
            self.cmd_vel.publish(move_cmd)
            r.sleep()

        print(ticks)
        self.cmd_vel.publish(Twist())#停止
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
if __name__ == '__main__':
    try:
        OutAndBack()
    except:
        rospy.loginfo("Out-and-Back node terminated.")