cmake_minimum_required(VERSION 3.3)
project(front_end)
SET(CMAKE_CXX_COMPILER "g++")


# 安装第三方库
find_package(Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIR})
link_directories(${EIGEN3_LIBRARY_DIRS})

find_package(PCL COMPONENTS visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# 安装lv_math头文件, !注意修改此文件路径
set(LV_MATH_INSTALL_DIR ${PROJECT_SOURCE_DIR}/../lv_math/install)

message(STATUS "lv_math install dir: ${LV_MATH_INSTALL_DIR}")

include_directories(${LV_MATH_INSTALL_DIR}/include/math)

# 编译
if(COMPILE_MODE MATCHES "TEST")
    add_subdirectory(src/test)
else()
    add_subdirectory(src/core)
endif()