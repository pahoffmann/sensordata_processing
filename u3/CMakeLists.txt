cmake_minimum_required(VERSION 2.8)
project( Stereo )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( StereoCalib stereo_calib.cpp )
target_link_libraries( StereoCalib ${OpenCV_LIBS} )

# add_executable( StereoMatch stereo_match.cpp )
# target_link_libraries( StereoMatch ${OpenCV_LIBS} )