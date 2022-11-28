PROJECT(third_party)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTORCC ON)
SET(CMAKE_AUTOUIC ON)

SET(CMAKE_CXX_FLAGS "-march=native -Wall -Werror -g -Wno-unused-function")
set(CMAKE_CXX_STANDARD 14)

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -fno-builtin-malloc \
      -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free -O3 -DNDEBUG")
  SET(BUILD_SPECIFIC_LIBRARIES "tcmalloc")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
  SET(BUILD_SPECIFIC_LIBRARIES "")
ENDIF()

MESSAGE(STATUS "SOURCE DIR: ${PROJECT_SOURCE_DIR}")

# Qt setup (needs to precede executable)
set(CMAKE_AUTOMOC on)
set(CMAKE_AUTORCC on)
set(CMAKE_AUTOUIC on)
set(CMAKE_INCLUDE_CURRENT_DIR on)
find_package(Qt5 COMPONENTS Core Widgets WebSockets Gui OpenGL REQUIRED)

FIND_PACKAGE(Lua51 REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

ROSBUILD_GENMSG()
ROSBUILD_GENSRV()

INCLUDE_DIRECTORIES(vector_display)
INCLUDE_DIRECTORIES(vector_display/src)
INCLUDE_DIRECTORIES(vector_display/src/gui)
INCLUDE_DIRECTORIES(vector_display/src/vector_display)
INCLUDE_DIRECTORIES(vector_display/src/third_party)
INCLUDE_DIRECTORIES(vector_display/src/config_reader/include)


INCLUDE_DIRECTORIES(${Qt5Widgets_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Qt5Gui_INCLUDE_DIRS})


SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})

ADD_SUBDIRECTORY(amrl_shared_lib)

SET(libs roslib roscpp pthread glut gflags
    Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL
    ${LUA_LIBRARIES} rt GL GLU
    glog X11 roscpp roslib amrl_shared_lib)


ROSBUILD_ADD_EXECUTABLE(curiosity
    enml/src/curiosity_main.cpp
    enml/src/non_markov_localization.cpp
    enml/src/kdtree.cpp
    enml/src/perception_2d.cpp
    enml/src/vector_map/vector_map.cc
    enml/src/visualization/visualization.cc
    enml/src/visualization/gui_helpers.cc)
TARGET_INCLUDE_DIRECTORIES(curiosity PRIVATE
    amrl_shared_lib
    config-reader/include
    enml/src
    enml/src/shared
    enml/src/config-reader/include
    ${LUA_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS})
ADD_DEPENDENCIES(curiosity rosbuild_precompile)
TARGET_LINK_LIBRARIES(curiosity ceres roslib roscpp ncurses pthread
    boost_system ${LUA_LIBRARIES} rt popt glog rosbag amrl_shared_lib
    ${BUILD_SPECIFIC_LIBRARIES})

# SET(target vector_display_turtlebot)
# ROSBUILD_ADD_EXECUTABLE(${target}
#     vector_display/src/gui/vector_display_main.cpp
#     vector_display/src/gui/vector_display_thread_turtlebot.cpp
#     vector_display/src/gui/vector_display.cpp
#     vector_display/src/gui/gltext.cpp
#     vector_display/src/vector_map/vector_map.cc
#     ${THIRD_PARTY_MOC_SRCS})
# TARGET_INCLUDE_DIRECTORIES(${target} PRIVATE
#     vector_display/src
#     vector_display/src/shared
#     vector_display/src/vector_map
#     vector_display/src/gui)
#     TARGET_LINK_LIBRARIES(${target} ${libs} ${QT_LIBRARIES})

# SET(target2 vector_display_tb2)
# ROSBUILD_ADD_EXECUTABLE(${target2}
#     vector_display/src/gui/vector_display_main.cpp
#     vector_display/src/gui/vector_display_thread_tb2.cpp
#     vector_display/src/gui/vector_display.cpp
#     vector_display/src/gui/gltext.cpp
#     vector_display/src/vector_map/vector_map.cc
#     ${THIRD_PARTY_MOC_SRCS})
# TARGET_INCLUDE_DIRECTORIES(${target2} PRIVATE
#     vector_display/src
#     vector_display/src/shared
#     vector_display/src/vector_map
#     vector_display/src/gui)
#     TARGET_LINK_LIBRARIES(${target2} ${libs} ${QT_LIBRARIES})




