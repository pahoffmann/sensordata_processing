project( Calibration )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( Calibration calibrate.cpp )
target_link_libraries( Calibration ${OpenCV_LIBS} )

add_executable( Extrinsics extrinsics.cpp )
target_link_libraries( Extrinsics ${OpenCV_LIBS} )

add_executable( Scanner david_scanner.cpp culture_invariant_ply_writer.cpp)
target_link_libraries( Scanner ${OpenCV_LIBS} )