SET(PROJECT_NAME vikit_ros)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)

# Set build flags
SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g")
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

# Add catkin and required ROS packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  roscpp vikit_common visualization_msgs tf eigen_catkin sophus_catkin
)

# Add plain cmake packages 
FIND_PACKAGE(OpenCV REQUIRED)

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Describe catkin Project
catkin_package(
    DEPENDS OpenCV
    CATKIN_DEPENDS roscpp visualization_msgs tf vikit_common eigen_catkin sophus_catkin
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

# Set Sourcefiles
LIST(APPEND SOURCEFILES 
  src/output_helper.cpp
)

# Create vikit library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
  ${catkin_LIBRARIES})