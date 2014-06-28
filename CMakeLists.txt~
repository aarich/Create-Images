cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Render_Images)

find_package(PCL 1.7 REQUIRED)
find_package( OpenCV REQUIRED )


include_directories(${PCL_INCLUDE_DIRS})
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (render_test src/main.cpp)
target_link_libraries (render_test ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})


