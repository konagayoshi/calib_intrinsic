set( TARGET calib_chart_generator )

project(ProjectName)

cmake_minimum_required( VERSION 2.8 )

find_package( OpenCV 4 REQUIRED )

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

set( OPENCV_LIBS opencv_core opencv_imgproc opencv_aruco opencv_highgui )

add_executable( ${TARGET} ${SRCS}  )
target_link_libraries( ${TARGET} ${OPENCV_LIBS} )
