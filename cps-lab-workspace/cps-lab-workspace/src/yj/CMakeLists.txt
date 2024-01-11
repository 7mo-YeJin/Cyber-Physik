cmake_minimum_required(VERSION 2.8.3)
project(yj)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

# IMPORTANT: you must copy and adapt the following lines for each node you want to add!
add_executable(Publisher src/publisher.cpp)
target_link_libraries(Publisher ${catkin_LIBRARIES})
add_dependencies(Publisher yj_generate_messages_cpp)

# IMPORTANT: you must copy and adapt the following lines for each node you want to add!
add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber ${catkin_LIBRARIES})
add_dependencies(subscriber yj_generate_messages_cpp)

# IMPORTANT: you must copy and adapt the following lines for each node you want to add!
add_executable(emitter src/emitter.cpp)
target_link_libraries(emitter ${catkin_LIBRARIES})
add_dependencies(emitter yj_generate_messages_cpp)

# IMPORTANT: you must copy and adapt the following lines for each node you want to add!
add_executable(receiver src/receiver.cpp)
target_link_libraries(receiver ${catkin_LIBRARIES})
add_dependencies(receiver yj_generate_messages_cpp)

# IMPORTANT: you must copy and adapt the following lines for each node you want to add!
add_executable(user_input src/user_input.cpp)
target_link_libraries(user_input ${catkin_LIBRARIES})
add_dependencies(user_input yj_generate_messages_cpp)
