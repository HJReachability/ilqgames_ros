cmake_minimum_required(VERSION 2.8.3)
project(ilqgames_ros)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/ilqgames_ros/cmake)
message("Cmake module path: ${CMAKE_MODULE_PATH}")

find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)

#include(ExternalProject)
#include("${CMAKE_SOURCE_DIR}/ilqgames_ros/cmake/gflags.cmake")
#include_directories(SYSTEM ${GFLAGS_INCLUDE_DIRS})
#list(APPEND catkin_LIBRARIES ${GFLAGS_LIBRARIES})

#include("${CMAKE_SOURCE_DIR}/ilqgames_ros/cmake/glog.cmake")
#include_directories(SYSTEM ${GLOG_INCLUDE_DIRS})
#list(APPEND catkin_LIBRARIES ${GLOG_LIBRARIES})



find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  geometry_msgs
  std_msgs
  darpa_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    visualization_msgs
    geometry_msgs
    std_msgs
    darpa_msgs
  DEPENDS
    EIGEN3
    GLOG
    ilqgames
)

option(ENABLE_DEBUG_MESSAGES "Turn on to enable bound checking and debug messages" ON)
if(ENABLE_DEBUG_MESSAGES)
  add_definitions(-DENABLE_DEBUG_MESSAGES=1)
else()
  add_definitions(-DENABLE_DEBUG_MESSAGES=0)
endif()

set(ILQGAMES_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/../../external/install_isolated/include/ilqgames)

add_definitions(-DILQGAMES_ROS_LOG_DIR="${CMAKE_SOURCE_DIR}/ilqgames_ros/logs/")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIRS}
  ${ILQGAMES_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${EIGEN3_LIBRARY_DIRS}
  ${GLOG_LIBRARY_DIRS}
  ${ILQGAMES_LIBRARY_DIRS}
)

file(GLOB_RECURSE ${PROJECT_NAME}_srcs ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src/*.cpp)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_srcs})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${ILQGAMES_LIBRARIES}
)
endif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")

file(GLOB_RECURSE ${PROJECT_NAME}_execs exec/*.cpp)

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
foreach(executable ${${PROJECT_NAME}_execs})
  get_filename_component(exec_name ${executable} NAME_WE)
  message(STATUS "Including executable   \"${BoldBlue}${exec_name}${ColorReset}\".")
  add_executable(${exec_name} ${executable})
  add_dependencies(${exec_name} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${exec_name}
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${ILQGAMES_LIBRARIES}
    ${CMAKE_SOURCE_DIR}/../../external/install_isolated/lib/libilqgames.a
    )
endforeach()
endif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
