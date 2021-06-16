cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 11)

include_directories(${EIGEN_INCLUDE_DIR})
include_directories( "/usr/local/include/eigen3" )
include_directories( "include" )

# set the project name
project(makepro)

# add the executable
add_executable(Tutorial src/tutorial.cpp)