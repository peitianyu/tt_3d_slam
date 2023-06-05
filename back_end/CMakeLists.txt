cmake_minimum_required(VERSION 3.3)
project(back_end)
SET(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG -Wall -std=c++14 -fPIC ")

# 安装第三方库
find_package(Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIR})
link_directories(${EIGEN3_LIBRARY_DIRS})

find_package(PCL COMPONENTS visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

# 安装lv_math头文件, !注意修改此文件路径
set(LV_MATH_INSTALL_DIR ${PROJECT_SOURCE_DIR}/../lv_math/install)
set(LV_PLATFORM_INSTALL_DIR ${PROJECT_SOURCE_DIR}/../lv_platform/install)
set(LV_VIZ_INSTALL_DIR ${PROJECT_SOURCE_DIR}/../viz/install)
set(LV_FRONT_END_INSTALL_DIR ${PROJECT_SOURCE_DIR}/../front_end/install/)

message(STATUS "lv_math install dir: ${LV_MATH_INSTALL_DIR}")

include_directories(${LV_MATH_INSTALL_DIR}/include/math)
include_directories(${LV_PLATFORM_INSTALL_DIR}/include)
include_directories(${LV_VIZ_INSTALL_DIR}/include)
include_directories(${LV_FRONT_END_INSTALL_DIR}/include/src)

# 编译
if(COMPILE_MODE MATCHES "TEST")
    add_subdirectory(src/test)
else()
    add_subdirectory(src/core)
endif()