SET(PROJECT_NAME vikit_cameras)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)

# Options
OPTION(VIKIT_BUILD_GPL_LICENSED_CODE "Should GPL licensed code be build in the library" ON)
OPTION(VIKIT_USE_ROS "Is ROS available?" ON)

SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

# Set build flags. Set IS_ARM on odroid board as environment variable
SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
IF(DEFINED ENV{ARM_ARCHITECTURE})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
ENDIF()
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Add plain cmake packages
#FIND_PACKAGE(OpenCV REQUIRED)

# Include dirs
INCLUDE_DIRECTORIES(
  include
  #${OpenCV_INCLUDE_DIRS}
)

# Set link libraries
LIST(APPEND VIKIT_LINK_LIBS 
  #${OpenCV_LIBS}
)

# TODO, make option:
ADD_DEFINITIONS(-DVIKIT_USE_YAML)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
  eigen_catkin sophus_catkin yaml_cpp_catkin
)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
LIST(APPEND VIKIT_LINK_LIBS ${catkin_LIBRARIES})
catkin_package(
  DEPENDS YamlCpp
  CATKIN_DEPENDS eigen_catkin sophus_catkin yaml_cpp_catkin
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
)

# Create vikit shared library
ADD_LIBRARY(${PROJECT_NAME} SHARED
  include/vikit/cameras.h
  include/vikit/cameras/camera_geometry_base.h
  include/vikit/cameras/no_distortion.h
  include/vikit/cameras/atan_distortion.h
  include/vikit/cameras/equidistant_distortion.h
  include/vikit/cameras/radial_tangential_distortion.h
  include/vikit/cameras/pinhole_projection.h
  include/vikit/cameras/implementation/pinhole_projection.hpp
  include/vikit/cameras/camera_geometry.h
  include/vikit/cameras/implementation/camera_geometry.hpp
  include/vikit/cameras/camera_factory.h
  src/camera_factory.cpp
)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${VIKIT_LINK_LIBS})

# Tests
catkin_add_gtest(${PROJECT_NAME}_tests
  test/test_cameras
)
target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME} ${Boost_LIBRARIES})
