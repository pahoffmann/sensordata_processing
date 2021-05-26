project( Stereo )
set(DATA_DIR "${CMAKE_SOURCE_DIR}/u3/data/")
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( StereoCalib capture_images.cpp )
target_link_libraries( StereoCalib ${OpenCV_LIBS} )


if (NOT EXISTS "${CMAKE_SOURCE_DIR}/opencv/build/bin/example_cpp_stereo_calib")
    message(WARNING "Can't symlink stereo calibration ${CMAKE_SOURCE_DIR}. Have you compiled opencv with 'cmake -DBUILD_EXAMPLES=ON ..'?")
else()
    if (NOT EXISTS "${DATA_DIR}")
        file(MAKE_DIRECTORY "${DATA_DIR}")
    endif()
    file(COPY "${CMAKE_SOURCE_DIR}/opencv/build/bin/example_cpp_stereo_calib" 
         DESTINATION "${DATA_DIR}")
    file(COPY "${CMAKE_SOURCE_DIR}/opencv/samples/data/stereo_calib.xml" 
         DESTINATION "${DATA_DIR}")
endif()