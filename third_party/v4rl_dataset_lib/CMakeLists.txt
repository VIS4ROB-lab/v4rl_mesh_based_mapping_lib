cmake_minimum_required(VERSION 2.8.11)

project(v4rl_dataset_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++11 -fPIC -mssse3")

FIND_PACKAGE(Boost COMPONENTS filesystem system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(include)

FILE(
  GLOB SRCS 
  src/*.cpp
)

FILE(
  GLOB HPPS 
  include/*.hpp
)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )

add_library(v4rl_dataset_lib ${SRCS} ${HPPS})


add_executable(test_asl_dataset_helper test/test_asl_dataset_helper.cpp src/asl_dataset_helper.cpp include/asl_dataset_helper.h)

target_link_libraries(test_asl_dataset_helper
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    pthread
    yaml-cpp
)
