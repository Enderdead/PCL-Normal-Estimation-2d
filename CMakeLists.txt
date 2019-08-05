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
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )


#PointXYNormal.cpp PointXYNormal.h
add_executable(norm_2d main.cpp include/PCA2D.h src/PCA2D.cpp include/Normal2dEstimation.h src/Normal2dEstimation.cpp)
target_link_libraries(norm_2d ${PCL_LIBRARIES}  ${Boost_LIBRARIES})
