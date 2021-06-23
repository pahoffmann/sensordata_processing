project( marchingCubes )

find_package( flann REQUIRED )
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package ( Boost 1.40 COMPONENTS program_options REQUIRED )
include_directories( ${Boost_INCLUDE_DIR} )
include_directories( /usr/local/include )

set(SOURCES octree.cpp distance_function.cpp)
add_executable( octree_test test_octree.cpp octree.cpp)
target_link_libraries(octree Eigen3::Eigen)

# add_executable( estimate_normals estimate_normals.cpp kdTree.cpp culture_invariant_ply_writer.cpp )
# target_link_libraries (estimate_normals Eigen3::Eigen ${Boost_LIBRARIES})

set(CMAKE_CXX_FLAGS "-O3")