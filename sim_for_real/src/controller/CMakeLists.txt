cmake_minimum_required(VERSION 2.8.3)
project(controller)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

# IMPORTANT: you must copy and adapt the following lines for each node you want to add!
add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})
add_dependencies(controller controller_generate_messages_cpp)

add_executable(inputmux src/inputmux.cpp)
target_link_libraries(inputmux ${catkin_LIBRARIES})
add_dependencies(inputmux inputmux_generate_messages_cpp)

add_executable(keyboard_teleop src/keyboard_teleop.cpp)
target_link_libraries(keyboard_teleop ${catkin_LIBRARIES})
add_dependencies(keyboard_teleop keyboard_teleop_generate_messages_cpp)

add_executable(aeb src/aeb.cpp)
target_link_libraries(aeb ${catkin_LIBRARIES})
add_dependencies(aeb aeb_generate_messages_cpp)
