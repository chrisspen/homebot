cmake_minimum_required(VERSION 2.8.3)
project(ros_homebot)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    message_generation
    std_msgs
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
  CATKIN_DEPENDS actionlib_msgs
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY nodes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
