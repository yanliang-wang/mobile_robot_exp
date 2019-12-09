cmake_minimum_required(VERSION 2.8.3)
project(cmd_pkg)
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Debug")
find_package(catkin REQUIRED COMPONENTS 
        roscpp
        rospy
        std_msgs
        moveit_ros_planning_interface
        moveit_msgs
        tf
        tf2_ros
        tf2_geometry_msgs
        geometry_msgs
        hand_control
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        geometry_msgs
)

include_directories(
        include
        /usr/include/eigen3  # include the header file of eigen3
        ${catkin_INCLUDE_DIRS}
)

add_executable(move_aubo
        src/move_aubo.cpp src/move_aubo_api.cpp)
add_dependencies(move_aubo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(move_aubo  ${catkin_LIBRARIES})

add_executable(get_pose
        src/get_pose.cpp)
add_dependencies(get_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_pose ${catkin_LIBRARIES})

add_executable(aubo_add_collision
        src/aubo_add_collision.cpp )
add_dependencies(aubo_add_collision ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(aubo_add_collision  ${catkin_LIBRARIES})

add_executable(move_all
        src/move_all.cpp src/move_aubo_api.cpp src/move_all_base.cpp)
add_dependencies(move_all ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}  hand_control_generate_messages_cpp)
target_link_libraries(move_all  ${catkin_LIBRARIES}  )

add_executable(publish_basic_target
        src/publish_basic_target.cpp)
add_dependencies(publish_basic_target ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_basic_target  ${catkin_LIBRARIES}  )