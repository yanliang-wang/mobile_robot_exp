cmake_minimum_required(VERSION 2.8.3)
project(hand_control)

set(SOURCE_FILES
        src/SerialClass.cpp
        src/main.cpp
src/main.cpp
        src/Protocol.cpp
	)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        serial
        std_msgs
        message_generation
)
add_message_files(
   FILES
   hand_control_cmd.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hand_control
        CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
   include
)

add_executable(${PROJECT_NAME}_node ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME}_node hand_control_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(talker src/talker.cpp)
add_dependencies(talker hand_control_generate_messages_cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

