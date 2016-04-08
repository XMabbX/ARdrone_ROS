cmake_minimum_required(VERSION 2.8.3)
project(intros_action_example)

find_package(catkin REQUIRED COMPONENTS  message_generation roscpp rospy std_msgs actionlib actionlib_msgs)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(  ${catkin_INCLUDE_DIRS} )

add_action_files(
  DIRECTORY action
  FILES Fibonacci.action
)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package( 
#  INCLUDE_DIRS include
#  LIBRARIES action
 CATKIN_DEPENDS roscpp rospy std_msgs actionlib_msgs message_runtime
  DEPENDS Boost 
)

add_executable(action_server_node src/server_node.cpp)
target_link_libraries(action_server_node ${catkin_LIBRARIES})

add_executable(action_client_node src/client_node.cpp)
target_link_libraries(action_client_node ${catkin_LIBRARIES})