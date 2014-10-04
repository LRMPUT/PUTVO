cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(iterative)

set(CMAKE_BUILD_TYPE Release) 
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") # -Wall 

find_package(PCL)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (kinect Includes/functions.cpp DBscan/dbscan.cpp Quadtree/lcg.cpp Quadtree/quadtree.cpp Show/Show.cpp TransformationEstimation/TransformationEstimation.cpp Track/track.cpp Filtry/Filtry.cpp RGBD/RGBDprocessing.cpp kinect.cpp  main.cpp )
target_link_libraries (kinect ${PCL_LIBRARIES} ${OpenCV_LIBS})
