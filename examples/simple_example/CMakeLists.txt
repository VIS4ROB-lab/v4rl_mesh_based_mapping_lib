
ADD_DEFINITIONS (-march=native -std=c++11 )


include_directories(${CMAKE_SOURCE_DIR}/third_party/fade_v1.46a/include_fade2d)
include_directories(${CMAKE_SOURCE_DIR}/third_party/okvis_kinematics/include)
add_executable(simple_example src/main.cpp ${CMAKE_SOURCE_DIR}/include/mesh_based_mapping.hpp)

target_link_libraries(simple_example
                        ${OpenCV_LIBS}
                        ${CMAKE_SOURCE_DIR}/third_party/fade_v1.46a/lib_ubuntu14.04_x86_64/libfade2d.so )

