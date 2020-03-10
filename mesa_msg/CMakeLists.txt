cmake_minimum_required(VERSION 2.8.3)
project(mesa_msg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   Mesa.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
      std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_custom_service_msg
  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
