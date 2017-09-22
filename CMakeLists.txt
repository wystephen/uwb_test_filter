cmake_minimum_required(VERSION 3.8)
project(uwb_test_filter)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11   -pthread -lopencv_aruco")


find_package(Eigen3 REQUIRED)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ./include
        /usr/local/cuda/include
        /usr/include
        /usr/local/include

)


set(SOURCE_FILES main.cpp)
add_executable(uwb_test_filter ${SOURCE_FILES})