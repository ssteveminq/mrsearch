cmake_minimum_required(VERSION 2.8.3)
project(gmm)
set(MY_LIB_SOURCES
    src/Utils.cpp
    src/GaussianDistribution.cpp
    src/GaussianDistributionFactory.cpp
    src/GaussianDistributionSampler.cpp
    src/GaussianMixtureModelSampler.cpp
    src/GaussianMixtureModel.cpp
    src/GaussianMixtureModelFactory.cpp
    src/ExpectationMaximization.cpp
    src/TrainSet.cpp
    src/EvaluatioLogDensity.cpp
    src/KMeans.cpp
)



find_package(catkin REQUIRED COMPONENTS
    roslib
    roscpp
    rospy
    std_msgs
    geometry_msgs
    tf
    nav_msgs
    visualization_msgs
    actionlib
    actionlib_msgs
)


find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
    set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

add_action_files(
  DIRECTORY action
  FILES
  Prediction.action
)

generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
)




include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    #inlcude/GaussianUtils
    #inlcude/GaussianMixtureModel
    )


catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    std_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    actionlib
    actionlib_msgs
)
#)

#GaussianUtils
#)

add_library(gmm_library ${MY_LIB_SOURCES})
target_link_libraries(gmm_library ${catkin_LIBRARIES})

add_executable(test_sampling src/sample_test.cpp)
target_link_libraries(test_sampling ${catkin_LIBRARIES} gmm_library )
add_dependencies(test_sampling ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_executable(human_belief src/human_belief.cpp)
add_dependencies(human_belief ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(human_belief ${catkin_LIBRARIES} gmm_library)



install(TARGETS gmm_library
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)


install(DIRECTORY include/${PROJECT_SHORTNAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
 
install(DIRECTORY include/${PROJECT_SHORTNAME2}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
 
install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)


