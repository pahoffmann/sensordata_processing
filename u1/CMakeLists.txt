cmake_minimum_required(VERSION 2.8)
project( David )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( Calibration_David calibrate.cpp )
target_link_libraries( Calibration_David ${OpenCV_LIBS} )

add_executable( Extrinsics_David extrinsics.cpp )
target_link_libraries( Extrinsics_David ${OpenCV_LIBS} )