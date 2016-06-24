cmake_minimum_required(VERSION 2.8.3)
project(cutplan)

set(MSG_FILES
  Dummy.msg
)

set(SRV_FILES
  GetManeuver.srv
)

set(ACT_FILES
)

#add_action_files(DIRECTORY action FILES ${ACT_FILES})
add_message_files(DIRECTORY msg FILES ${MSG_FILES})
add_service_files(DIRECTORY srv FILES ${SRV_FILES})

find_package(PkgConfig)
find_package(catkin REQUIRED COMPONENTS
  meshproc_msgs
  std_msgs
  shape_msgs
  message_generation
  roslisp
)

include_directories(${catkin_INCLUDE_DIRS})

generate_messages(DEPENDENCIES std_msgs shape_msgs meshproc_msgs)

catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES
  CATKIN_DEPENDS roslisp std_msgs meshproc_msgs shape_msgs message_runtime
)

