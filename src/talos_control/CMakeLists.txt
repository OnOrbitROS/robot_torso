cmake_minimum_required(VERSION 3.0.2)
project(talos_control)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set ( CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/modules/  )
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")
add_definitions("-DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS")

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  kdl_parser
  pluginlib
  roscpp
  rospy
  geometry_msgs
  trajectory_msgs
  message_generation
) 

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  target_frame.msg
  plot_msg.msg
  plot_jnt.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  hand_operations.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  trajectory_msgs
)




catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES talos_control
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
 DEPENDS Boost 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${pinocchio_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)



add_library(talos_control_lib src/cartesian_controller.cpp)
add_dependencies(talos_control_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(talos_control_lib ${catkin_LIBRARIES} ${pinocchio_LIBRARIES} ${boost_LIBRARIES} ${Eigen3_LIBRARIES})

add_executable(hand_service src/hand_service.cpp)
target_link_libraries(hand_service ${catkin_LIBRARIES})
add_dependencies(hand_service talos_control_gencpp)

add_executable(gripper_service src/gripper_service.cpp)
target_link_libraries(gripper_service ${catkin_LIBRARIES})
add_dependencies(gripper_service talos_control_gencpp)

add_executable(matrix_transf src/matrix_transf.cpp)
target_link_libraries(matrix_transf ${catkin_LIBRARIES})

add_executable(trajectories src/trajectories.cpp)
target_link_libraries(trajectories ${catkin_LIBRARIES})

add_executable(pinocchio_jacobiana src/pinocchio_jacobiana.cc)
target_link_libraries(pinocchio_jacobiana ${pinocchio_LIBRARIES} ${boost_LIBRARIES})

add_executable(look_to_aruco src/look_to_aruco.cpp)
target_link_libraries(look_to_aruco ${catkin_LIBRARIES})
add_dependencies(look_to_aruco talos_control_gencpp)
