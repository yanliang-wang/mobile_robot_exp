cmake_minimum_required(VERSION 2.8.3)
project(cmd_pkg)
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Debug")
find_package(catkin REQUIRED COMPONENTS 
        roscpp
        rospy
        std_msgs
        moveit_core
        moveit_visual_tools
        moveit_ros_planning
        moveit_ros_planning_interface
        pluginlib
        geometric_shapes
        pcl_ros
        pcl_conversions
        rosbag
        tf
        tf2_ros
        tf2_geometry_msgs
        geometry_msgs
        moveit_msgs

)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_aubo src/move_aubo.cpp src/move_aubo_api.cpp)
add_dependencies(move_aubo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(move_aubo
  ${catkin_LIBRARIES}
)

add_executable(get_pose
  src/get_pose.cpp
)
add_dependencies(get_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_pose
  ${catkin_LIBRARIES}
)

add_executable(aubo_add_collision src/aubo_add_collision.cpp )
add_dependencies(aubo_add_collision ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(aubo_add_collision
        ${catkin_LIBRARIES}
        )