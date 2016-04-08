cmake_minimum_required(VERSION 2.8.3)
project(intro_services_example)

find_package(catkin REQUIRED COMPONENTS  message_generation roscpp rospy std_msgs )

add_service_files(DIRECTORY srv FILES service_intro.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime roscpp rospy std_msgs)

include_directories(  ${catkin_INCLUDE_DIRS} )

add_executable(server_node src/server_node.cpp)
target_link_libraries(server_node ${catkin_LIBRARIES})

add_executable(client_node src/client_node.cpp)
target_link_libraries(client_node ${catkin_LIBRARIES})


