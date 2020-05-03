set( TARGET calib_intrinsic )
set( RASPICAM_DIR /opt/raspivideocap )

cmake_minimum_required( VERSION 2.8 )

find_package( OpenCV 3 REQUIRED )

include_directories( ${RASPICAM_DIR}/include src )
link_directories( /opt/vc/lib ${RASPICAM_DIR}/lib )

file( GLOB SRCS src/*.cpp )

# to enable debug mode (!!Note: Clean build directory before changing mode)
#  > cmake -Ddebug=1 ..
if( debug )
  set( CMAKE_BUILD_TYPE Debug )
else()
  set( CMAKE_BUILD_TYPE Release )
endif()

set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ofast -DNDEBUG -march=native" )
set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g3" )

set( MMAL_LIBS mmal_core mmal mmal_util vcos bcm_host )
set( OPENCV_LIBS opencv_core opencv_imgproc opencv_calib3d opencv_aruco opencv_ccalib opencv_highgui )

add_executable( ${TARGET} ${SRCS}  )
target_link_libraries( ${TARGET} ${MMAL_LIBS} raspivideocap ${OPENCV_LIBS} )
