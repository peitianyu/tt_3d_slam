cmake_minimum_required(VERSION 3.3)
project(viz)
SET(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG -Wall -std=c++14 -fPIC -pthread")

# 安装第三方库
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
link_directories(${EIGEN3_LIBRARY_DIRS})

find_package(PCL COMPONENTS visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

set(LV_MATH_INSTALL_DIR ${PROJECT_SOURCE_DIR}/../lv_math/install)

# 编译
if(COMPILE_MODE MATCHES "TEST")
    add_subdirectory(src/test)
else()
    add_subdirectory(src/core)
endif()
