cmake_minimum_required(VERSION 2.8)
project( Calibration )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( Calibration calibrate.cpp )
target_link_libraries( Calibration ${OpenCV_LIBS} )

add_executable( Extrinsics extrinsics.cpp )
target_link_libraries( Extrinsics ${OpenCV_LIBS} )