cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_generation
  cv_bridge
)

find_package(OpenCV REQUIRED)
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Arg.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddTwoInts.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
)


## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS}, ${OpenCV_INCLUDE_DIRS})
#include_directories(include ${catkin_INCLUDE_DIRS}, /usr/local/include/opencv4)
#link_directories(/usr/local/lib)

	#	CATKIN_DEPENDS message_runtime
	#	INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib

add_executable(imagepub src/imagepub.cpp)
target_link_libraries(imagepub  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#target_link_libraries(imagepub ${catkin_LIBRARIES} opencv_core opencv_highgui)
add_dependencies(imagepub  beginner_tutorials_generate_messages_cpp)

