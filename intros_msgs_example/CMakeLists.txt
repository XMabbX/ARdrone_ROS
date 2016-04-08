cmake_minimum_required(VERSION 2.8.3)
project(intros_msgs_example)

find_package(catkin REQUIRED COMPONENTS  message_generation roscpp rospy std_msgs )

add_message_files(DIRECTORY msg FILES msg_introros.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime roscpp rospy std_msgs)

include_directories(  ${catkin_INCLUDE_DIRS} )



