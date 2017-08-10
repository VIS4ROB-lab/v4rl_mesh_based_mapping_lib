cmake_minimum_required(VERSION 2.8.0)
project(v4rl_mesh_based_mapping_lib)

set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)#TODO remove

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(Boost_USE_MULTITHREADED ON)

find_package( Eigen3 REQUIRED )
find_package( OpenCV REQUIRED )

include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories( ${OpenCV_INCLUDE_DIRS} )


include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )

add_subdirectory(examples/simple_example)

