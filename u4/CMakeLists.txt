project( kdTree )

find_package( flann REQUIRED )
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package ( Boost 1.40 COMPONENTS program_options REQUIRED )
include_directories( ${Boost_INCLUDE_DIR} )
include_directories( /usr/local/include )

set(SOURCES buildTree.cpp kdTree.cpp)
add_executable( kdTreeTest buildTree.cpp kdTree.cpp)
target_link_libraries(kdTreeTest /usr/local/lib/libflann.so)

add_executable( estimate_normals estimate_normals.cpp kdTree.cpp culture_invariant_ply_writer.cpp )
target_link_libraries (estimate_normals Eigen3::Eigen ${Boost_LIBRARIES})

set(CMAKE_CXX_FLAGS "-O3")