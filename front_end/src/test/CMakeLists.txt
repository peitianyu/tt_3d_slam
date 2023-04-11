file(GLOB_RECURSE ALL_LIBRIRY_SRCS "../core/*.cc")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG -Wall -std=c++14 -fPIC ")

# 选择关闭打印
# add_definitions(-DOFF_SLAM_LOG)

# 添加头文件
INCLUDE_DIRECTORIES(../core/src)

# 执行文件
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/install/test)

