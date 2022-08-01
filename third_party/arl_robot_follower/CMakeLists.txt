cmake_minimum_required(VERSION 3.0.2)
project(arl_robot_follower)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib_msgs
  actionlib
  #   arl_nav_msgs
  message_generation
  nav_msgs
  std_msgs
  std_srvs
  sensor_msgs
  tf
  tf2
  tf2_geometry_msgs
  vision_msgs
)

add_action_files(
  DIRECTORY action
  FILES
    FollowTarget.action
    InspectTarget.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    actionlib_msgs
    actionlib
    # arl_nav_msgs
    nav_msgs
    std_msgs
    std_srvs
    sensor_msgs
    tf2
    tf2_geometry_msgs
    vision_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# add_executable(robot_follower_server src/robot_follower_server.cpp)
# target_link_libraries(robot_follower_server ${catkin_LIBRARIES} ${FOCUS_LIBRARY_NAME})
# add_dependencies(robot_follower_server ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

# add_executable(inspection_server src/inspection_server.cpp)
# target_link_libraries(inspection_server ${catkin_LIBRARIES} ${FOCUS_LIBRARY_NAME})
# add_dependencies(inspection_server ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

# add_executable(supervisor src/supervisor.cpp)
# target_link_libraries(supervisor ${catkin_LIBRARIES} ${FOCUS_LIBRARY_NAME})
# add_dependencies(supervisor ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})


## Installation
# install(TARGETS robot_follower_server inspection_server supervisor
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
