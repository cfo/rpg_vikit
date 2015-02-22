SET(PROJECT_NAME vikit_common)
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
FIND_PACKAGE(OpenCV REQUIRED)

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Set link libraries
LIST(APPEND VIKIT_LINK_LIBS 
  ${OpenCV_LIBS}
)

# TODO, make option:
ADD_DEFINITIONS(-DVIKIT_USE_YAML)

IF(VIKIT_USE_ROS)
  FIND_PACKAGE(catkin REQUIRED COMPONENTS
    roscpp eigen_catkin sophus_catkin yaml_cpp_catkin
  )
  INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
  LIST(APPEND VIKIT_LINK_LIBS ${catkin_LIBRARIES})
  catkin_package(
    DEPENDS OpenCV YamlCpp
    CATKIN_DEPENDS roscpp eigen_catkin sophus_catkin yaml_cpp_catkin
    INCLUDE_DIRS include 
    LIBRARIES ${PROJECT_NAME}
  )
ELSE()
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF()

# Set source files
LIST(APPEND VIKIT_SOURCEFILES
  src/math_utils.cpp
  src/vision.cpp
  src/performance_monitor.cpp
  src/robust_cost.cpp
  src/user_input_thread.cpp
  src/homography.cpp
)

# Set header files
LIST(APPEND VIKIT_HEADERFILES
  include/vikit/aligned_mem.h
  include/vikit/blender_utils.h
  include/vikit/file_reader_types.h
  include/vikit/math_utils.h
  include/vikit/nlls_solver.h
  include/vikit/nlls_solver_impl.hpp
  include/vikit/patch_score.h
  include/vikit/performance_monitor.h
  include/vikit/ringbuffer.h
  include/vikit/robust_cost.h
  include/vikit/sample.h
  include/vikit/timer.h
  include/vikit/user_input_thread.h
  include/vikit/vision.h
  include/vikit/homography_decomp.h
  include/vikit/homography.h
)

IF(VIKIT_BUILD_GPL_LICENSED_CODE)
	LIST(APPEND VIKIT_SOURCEFILES
	)
	# Set header files
	LIST(APPEND VIKIT_HEADERFILES
	  include/vikit/file_reader.h
	)
ENDIF()

# Create vikit shared library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${VIKIT_SOURCEFILES} ${VIKIT_HEADERFILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${VIKIT_LINK_LIBS})

# Unit tests
CATKIN_ADD_GTEST(homography test/test_homography.cpp)
TARGET_LINK_LIBRARIES(homography svo gtest)

# Other tests
ADD_EXECUTABLE(vikit_test_timer test/test_timer.cpp)
TARGET_LINK_LIBRARIES(vikit_test_timer ${PROJECT_NAME})

ADD_EXECUTABLE(vikit_test_patch_score test/test_patch_score.cpp)
TARGET_LINK_LIBRARIES(vikit_test_patch_score ${PROJECT_NAME})

################################################################################
# Create the vikit_commonConfig.cmake file for other cmake projects.
IF(NOT VIKIT_USE_ROS)
  GET_TARGET_PROPERTY( FULL_LIBRARY_NAME ${PROJECT_NAME} LOCATION )
  SET(vikit_common_LIBRARIES ${FULL_LIBRARY_NAME} )
  SET(vikit_common_LIBRARY_DIR ${PROJECT_BINARY_DIR} )
  SET(vikit_common_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
  CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/vikit_commonConfig.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}/vikit_commonConfig.cmake @ONLY IMMEDIATE )
  export( PACKAGE vikit_common )

  INSTALL(DIRECTORY include/vikit DESTINATION ${CMAKE_INSTALL_PREFIX}/include FILES_MATCHING PATTERN "*.h" )
  INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
ENDIF()
