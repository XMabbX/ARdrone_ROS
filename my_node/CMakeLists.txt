cmake_minimum_required(VERSION 2.8.3)
project(my_node)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs dynamic_reconfigure)

generate_dynamic_reconfigure_options(cfg/my_nodeparams.cfg)

catkin_package()

include_directories( ${catkin_INCLUDE_DIRS} )

add_executable(my_node src/my_node.cpp)

target_link_libraries(my_node ${catkin_LIBRARIES})

add_dependencies(my_node my_node_gencfg)
