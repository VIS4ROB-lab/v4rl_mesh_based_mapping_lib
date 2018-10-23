
ADD_DEFINITIONS (-march=native -std=c++11 )



find_package( Boost COMPONENTS system filesystem thread REQUIRED )


include_directories( ${Boost_INCLUDE_DIRS} )
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
                        ${CMAKE_SOURCE_DIR}/third_party/fade_v1.46a/${FADE_LIB_VERSION}/libfade2d.so )

