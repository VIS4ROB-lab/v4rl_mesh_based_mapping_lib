cmake_minimum_required(VERSION 2.8.0)
project(v4rl_mesh_based_mapping_lib)

#set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(Boost_USE_MULTITHREADED ON)

find_package( Eigen3 REQUIRED )
find_package( OpenCV REQUIRED core highgui) # contrib

include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories( ${OpenCV_INCLUDE_DIRS} )

ADD_DEFINITIONS (-march=native -std=c++11 )

add_library(${PROJECT_NAME} STATIC src/mesh_based_mapping.cpp ${HPPS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/third_party/fade_v1.46a/include_fade2d)

set(FADE_LIB_VERSION "lib_ubuntu16.10_x86_64")# lib_ubuntu16.10_x86_64 or lib_ubuntu14.04_x86_64 or lib_ubuntu14.04_i686


add_subdirectory(examples/simple_example)

add_subdirectory(examples/okvis_dataset_example)

