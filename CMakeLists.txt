cmake_minimum_required(VERSION 2.12)
project(visp_ros_coppeliasim)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  visp_ros
  kdl_parser
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(iir REQUIRED)
find_package(orocos_kdl REQUIRED)  
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES visp_ros_coppeliasim
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs visp_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(SRC_FRANKA_MODEL
  src/fakeFCI.cpp
  src/FrictionTorque.cpp
  src/CoriolisMatrix.cpp
  src/GravityVector.cpp
  src/MassMatrix.cpp
)
#set(SRC_FRANKA_CLASS
#  src/fakeFCI.cpp
#)

## Declare a C++ library
set(FRANKA_MODEL_LIBRARY franka_model)
#set(FRANKA_CLASS_LIBRARY franka_class)
add_library(${FRANKA_MODEL_LIBRARY} SHARED ${SRC_FRANKA_MODEL})
#add_library(${FRANKA_CLASS_LIBRARY} SHARED ${SRC_FRANKA_CLASS})

#set(liborocos_kdl /opt/ros/melodic/lib/liborocos-kdl.so)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(four_dots_target_non_sync src/FourDotsTarget_nonSync.cpp)
add_executable(ibvs_april_tag_sync src/IBVS_AprilTag_sync.cpp)
add_executable(ibvs_melines_sync src/IBVS_meLines_sync.cpp)
add_executable(fake_fci src/fake_fci_test.cpp)
add_executable(franka_joint_torque_control src/Franka_joint_torque_control.cpp)
add_executable(franka_cartesian_torque_control src/Franka_cartesian_torque_control.cpp)
add_executable(orientation_test src/orientation_test.cpp)
add_executable(franka_image_torque_control src/Franka_image_torque_control.cpp)
add_executable(franka_image_lines_control src/Franka_image_lines_control.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(
   four_dots_target_non_sync
   ${VISP_LIBRARIES}
   ${catkin_LIBRARIES}
   iir::iir
 )
 
 target_link_libraries(
   ibvs_april_tag_sync
   ${FRANKA_MODEL_LIBRARY}
   ${VISP_LIBRARIES}
   ${catkin_LIBRARIES}
   iir::iir
 )
 
 target_link_libraries(
   ibvs_melines_sync
   ${FRANKA_MODEL_LIBRARY}
   ${VISP_LIBRARIES}
   ${catkin_LIBRARIES}
   iir::iir
 )
  
 target_link_libraries(
   fake_fci
   ${VISP_LIBRARIES}
   ${catkin_LIBRARIES}
   ${FRANKA_MODEL_LIBRARY}
   ${orocos_kdl_LIBRARIES}
   iir::iir
 )
 
  target_link_libraries(
   franka_joint_torque_control
   ${FRANKA_MODEL_LIBRARY}
   ${VISP_LIBRARIES}
   ${catkin_LIBRARIES}
   iir::iir
  )
 
  target_link_libraries(
    franka_cartesian_torque_control
    ${FRANKA_MODEL_LIBRARY}
    ${VISP_LIBRARIES}
    ${catkin_LIBRARIES}
    iir::iir
  )
 
  target_link_libraries(
   franka_image_torque_control
   ${FRANKA_MODEL_LIBRARY}
   ${VISP_LIBRARIES}
   ${catkin_LIBRARIES}
   iir::iir
 )
 
  target_link_libraries(
   franka_image_lines_control
   ${FRANKA_MODEL_LIBRARY}
   ${VISP_LIBRARIES}
   ${catkin_LIBRARIES}
   iir::iir
 )

  target_link_libraries(
   orientation_test
   ${FRANKA_MODEL_LIBRARY}
   ${VISP_LIBRARIES}
   ${catkin_LIBRARIES}
   iir::iir
 )
