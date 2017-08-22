
ADD_DEFINITIONS (-march=native -std=c++11 )

find_package( Boost COMPONENTS system filesystem thread REQUIRED )
set(OpenCV_DIR /home/lucas/code/opencv-2.4.13.2_install/share/OpenCV)
find_package(OpenCV REQUIRED core highgui )

include_directories( ${Boost_INCLUDE_DIRS} )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${CMAKE_SOURCE_DIR}/third_party/minkindr/include)
include_directories(${CMAKE_SOURCE_DIR}/third_party/v4rl_dataset_lib/include)

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

