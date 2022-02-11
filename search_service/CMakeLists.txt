cmake_minimum_required(VERSION 2.8.3)
project(search_service)	

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  actionlib
  actionlib_msgs
  message_generation
  nav_msgs
  visual_perception
)


find_package(Eigen3 REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS system thread)

# add_service_files(
#    FILES
# )
add_action_files(
   DIRECTORY action 
   FILES 
   Search.action
   MultiSearch.action
   PathFollow.action
   TSPSolve.action
   SetSearchRegion.action
)

add_service_files(
    FILES
    SetPolygon.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   std_srvs
   actionlib_msgs
   geometry_msgs
   nav_msgs
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    )
include_directories(src)

catkin_package(
 #INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy
  DEPENDS Boost
  geometry_msgs
  sensor_msgs
  std_msgs
  message_runtime
  tf
  actionlib_msgs
  nav_msgs
  actionlib
 )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


#add_executable(tf_odom_switcher src/hsrb_odom_switcher.cpp ${SOURCES})
#target_link_libraries(tf_odom_switcher ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(tf_odom_switcher ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_executable(mtsp_server src/mtsp_server.cpp src/cluster.cpp
    src/map_util.cpp ${SOURCES})
target_link_libraries(mtsp_server ${catkin_LIBRARIES})
add_dependencies(mtsp_server ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_executable(cluster_test src/cluster_test.cpp src/cluster.cpp ${SOURCES})
target_link_libraries(cluster_test ${catkin_LIBRARIES} )
add_dependencies(cluster_test ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_executable(scaled_map_converter src/scaled_map_converter.cpp
    src/map_util.cpp ${SOURCES})
target_link_libraries(scaled_map_converter ${catkin_LIBRARIES})
add_dependencies(scaled_map_converter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_executable(plugin_client src/plugin_client.cpp)
target_link_libraries(plugin_client
  ${catkin_LIBRARIES}
)


#add_executable(tf_odom_publisher src/hsrb_odom_publisher.cpp ${SOURCES})
#target_link_libraries(tf_odom_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(tf_odom_publisher ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

 #catkin_python_setup()
install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

