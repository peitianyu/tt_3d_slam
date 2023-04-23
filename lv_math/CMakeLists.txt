cmake_minimum_required(VERSION 3.3)
project(lv_math)
SET(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG -Wall -std=c++14 -fPIC -pthread")

# 安装第三方库
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

link_directories(${EIGEN3_LIBRARY_DIRS})

# 编译
if(COMPILE_MODE MATCHES "TEST")
    add_subdirectory(src/test)
else()
    add_subdirectory(src/core)
endif()