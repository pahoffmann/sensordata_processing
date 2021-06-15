project( kdTree )

find_package( flann REQUIRED )
include_directories( /usr/local/include )

set(SOURCES buildTree.cpp kdTree.cpp)
add_executable( kdTreeTest buildTree.cpp kdTree.cpp)
target_link_libraries(kdTreeTest /usr/local/lib/libflann.so)

add_executable( estimate_normals estimate_normals.cpp)

set(CMAKE_CXX_FLAGS "-O3")