cmake_minimum_required(VERSION 2.8)
project( MCL )
find_package( OpenCV REQUIRED )
add_executable( MCL MCL.cpp )
target_link_libraries( MCL ${OpenCV_LIBS} )

