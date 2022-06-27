cmake_minimum_required(VERSION 2.8.3)
project(visual_perception)

# Look for bfl (Bayesian Filtering Library)
find_package(PkgConfig REQUIRED)
pkg_check_modules(BFL REQUIRED orocos-bfl)

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  tf
  message_filters
  sensor_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  pcl_ros
  pcl_conversions
  trajectory_msgs
  control_msgs
  visualization_msgs
  frontier_exploration
)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)

add_action_files(
  DIRECTORY action
  FILES
  Search.action
  UnknownSearch.action
  VisualUnknownSearch.action
  GetFrontier.action
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
   std_srvs
   actionlib_msgs
)



## Declare things to be passed to dependent projects
catkin_package(
INCLUDE_DIRS include
#CATKIN_DEPENDS
    DEPENDS
    roscpp
    actionlib
    geometry_msgs
    sensor_msgs
    std_msgs
    nav_msgs
    actionlib_msgs
    visualization_msgs
    message_runtime
    frontier_exploration
    tf
#LIBRARIES visual_perception
)

## Specify additional locations of header files
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Declare a cpp library
#add_library(visual_perception 
#)

## Declare a cpp executable
add_executable(prediction_server src/prediction_server.cpp)
add_executable(visible_manager src/visible_region.cpp)
add_executable(visual_prediction_server src/visual_prediction_server.cpp)

## Add cmake target dependencies of the executable/library

add_dependencies(prediction_server ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
add_dependencies(visible_manager visual_perception ${catkin_EXPORTED_TARGETS})
add_dependencies(visual_prediction_server ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(visual_prediction_server ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
target_link_libraries(prediction_server ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
target_link_libraries(visible_manager ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BFL_LIBRARIES})

#install(TARGETS ${PROJECT_NAME}
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

