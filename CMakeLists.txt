cmake_minimum_required(VERSION 3.14)
project(norm_2d)

set(CMAKE_CXX_STANDARD 11)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)



# PCL Package ##############################################################
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Boost Package ############################################################
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.58.0 COMPONENTS program_options filesystem REQUIRED)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} include)

add_library(pcl_norm_2d src/PCA2D.cpp src/Normal2dEstimation.cpp)
set_target_properties(pcl_norm_2d PROPERTIES PUBLIC_HEADER "include/Normal2dEstimation.h;include/PCA2D.h")
install(TARGETS pcl_norm_2d
  PUBLIC_HEADER DESTINATION include/pcl_normal_estimation_2d)

#PointXYNormal.cpp PointXYNormal.h
add_executable(norm_2d main.cpp)
target_link_libraries(norm_2d pcl_norm_2d ${PCL_LIBRARIES}  ${Boost_LIBRARIES})

