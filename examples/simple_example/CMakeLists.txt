
ADD_DEFINITIONS (-march=native -std=c++11 )



include_directories(${CMAKE_SOURCE_DIR}/third_party/okvis_kinematics/include)
add_executable(simple_example src/main.cpp ${CMAKE_SOURCE_DIR}/include/mesh_based_mapping/mesh_based_mapping.hpp)

target_link_libraries(simple_example
                        ${OpenCV_LIBS}
                        v4rl_mesh_based_mapping_lib
                        ${CMAKE_SOURCE_DIR}/third_party/fade_v1.46a/${FADE_LIB_VERSION}/libfade2d.so )

