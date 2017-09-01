
ADD_DEFINITIONS (-march=native -std=c++11 )

set(OpenCV_DIR /home/lucas/code/opencv-2.4.13.2_install/share/OpenCV)

#set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

#find_package( Eigen3 REQUIRED )
#include_directories( ${EIGEN3_INCLUDE_DIR} )

find_package( Boost COMPONENTS system filesystem thread REQUIRED )
#find_package(OpenCV REQUIRED core highgui )




include_directories( ${Boost_INCLUDE_DIRS} )
#include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${CMAKE_SOURCE_DIR}/third_party/minkindr/include )
include_directories( ${CMAKE_SOURCE_DIR}/third_party/v4rl_dataset_lib/include )

FILE(
  GLOB HPPS 
  src/*.hpp
)

FILE(
  GLOB CPPS
  src/*.cpp
)


add_executable(okvis_dataset_example ${CPPS} ${HPPS})

target_link_libraries(okvis_dataset_example
                        ${OpenCV_LIBS}
                        v4rl_mesh_based_mapping_lib
                        ${Boost_LIBRARIES}
                        glog
                        ${CMAKE_SOURCE_DIR}/third_party/fade_v1.46a/lib_ubuntu14.04_x86_64/libfade2d.so )

