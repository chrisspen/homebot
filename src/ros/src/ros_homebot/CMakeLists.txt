cmake_minimum_required(VERSION 2.8.3)
project(ros_homebot)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    message_generation
    std_msgs
    roscpp
    serial
)

add_action_files(
   DIRECTORY action
   FILES
   SpinHead.action
   Wander.action
   Forward.action
   Turn.action
   Pivot.action
   TTS.action
   Tone.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs serial std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(serial_echo_node src/serial_echo_node.cpp)

target_link_libraries(serial_echo_node ${catkin_LIBRARIES})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY nodes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
