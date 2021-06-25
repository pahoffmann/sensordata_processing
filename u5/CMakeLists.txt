project( marchingCubes )

find_package( flann REQUIRED )
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package ( Boost 1.40 COMPONENTS program_options REQUIRED )
include_directories( ${Boost_INCLUDE_DIR} )
include_directories( /usr/local/include )

set(SOURCES octree.cpp distance_function.cpp reconstruct.cpp)
add_executable( octree_test test_octree.cpp octree.cpp distance_function.cpp)
target_link_libraries(octree_test Eigen3::Eigen /usr/local/lib/libflann.so /usr/lib/x86_64-linux-gnu/liblz4.so)

add_executable( reconstruct reconstruct.cpp octree.cpp distance_function.cpp culture_invariant_ply_writer.cpp)
target_link_libraries( reconstruct Eigen3::Eigen /usr/local/lib/libflann.so /usr/lib/x86_64-linux-gnu/liblz4.so ${Boost_LIBRARIES})

# add_executable( estimate_normals estimate_normals.cpp kdTree.cpp culture_invariant_ply_writer.cpp )
# target_link_libraries (estimate_normals Eigen3::Eigen ${Boost_LIBRARIES})

set(CMAKE_CXX_FLAGS "-O3")