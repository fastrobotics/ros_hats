cmake_minimum_required(VERSION 3.0.2)
project(ros_hats)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage -fpermissive ${WARNING_FLAGS}")
if (${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL armv7l)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DARCHITECTURE_ARMV7L")
endif()
add_definitions(-std=c++11 -Wall -Wextra -Wno-switch)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  eros
  std_msgs
  sensor_msgs
)
find_package(GTest REQUIRED)
find_package(rostest REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS eros
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}

)
add_subdirectory(src)
add_subdirectory(nodes)
# Doxygen

# look for Doxygen package
find_package(Doxygen)

if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile.out)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    # Note: do not put "ALL" - this builds docs together with application EVERY TIME!
    add_custom_target( ros_hats_docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)