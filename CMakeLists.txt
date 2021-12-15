cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "$***REMOVED***CXX_FLAGS***REMOVED***")

project(camera_fusion)

find_package(OpenCV 4.5 REQUIRED)

include_directories($***REMOVED***OpenCV_INCLUDE_DIRS***REMOVED***)
link_directories($***REMOVED***OpenCV_LIBRARY_DIRS***REMOVED***)
add_definitions($***REMOVED***OpenCV_DEFINITIONS***REMOVED***)

# Executable for create matrix exercise
add_executable (2D_feature_tracking src/matching2D_Student.cpp src/MidTermProject_Camera_Student.cpp)
target_link_libraries (2D_feature_tracking $***REMOVED***OpenCV_LIBRARIES***REMOVED***)