cmake_minimum_required(VERSION 3.5)
project(gnss_comm)

# set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fPIC -g")

# Set C++ standard to C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find catkin build system
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

# Message files
file(GLOB_RECURSE message_files ${PROJECT_SOURCE_DIR}/msg/*.msg)
add_message_files(FILES 
    GnssTimeMsg.msg
    GnssEphemMsg.msg
    GnssGloEphemMsg.msg
    GnssMeasMsg.msg
    GnssObsMsg.msg
    GnssBestXYZMsg.msg
    GnssPVTSolnMsg.msg
    GnssSvsMsg.msg
    GnssTimePulseInfoMsg.msg
    StampedFloat64Array.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS Eigen3 Glog
)

# Set module path for custom find modules
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)

# Include directories
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
)

# Collect all source files
file(GLOB cpp_source_files ${PROJECT_SOURCE_DIR}/src/*.cpp)
# Exclude gnss_ros.cpp to avoid ROS message dependency issues
list(FILTER cpp_source_files EXCLUDE REGEX ".*/gnss_ros\\.cpp$")
list(FILTER cpp_source_files EXCLUDE REGEX ".*/gnss_rtk\\.cpp$")

# Create main library
add_library(${PROJECT_NAME} ${cpp_source_files})
target_include_directories(${PROJECT_NAME} PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARIES}
)

# Add RTK positioning executable
add_executable(gnss_rtk 
  ${PROJECT_SOURCE_DIR}/src/gnss_rtk.cpp
  ${PROJECT_SOURCE_DIR}/src/gnss_ros.cpp
)
target_include_directories(gnss_rtk PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/
)
target_link_libraries(gnss_rtk ${PROJECT_NAME})
add_dependencies(gnss_rtk ${PROJECT_NAME}_generate_messages_cpp)
