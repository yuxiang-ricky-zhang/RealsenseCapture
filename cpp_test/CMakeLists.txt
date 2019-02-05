#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(capture)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(DEPENDENCIES realsense2)

# include(CheckIncludeFileCXX)

find_package(OpenCV REQUIRED)

add_executable(
	capture
	capture.cpp
)

target_link_libraries(capture 
						${DEPENDENCIES}
						${OpenCV_LIBRARIES})