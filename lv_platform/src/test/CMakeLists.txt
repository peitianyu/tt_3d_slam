file(GLOB_RECURSE ALL_LIBRIRY_SRCS ${PROJECT_SOURCE_DIR}/src/core/src/*.cc)

# 添加.h文件
include_directories(${PROJECT_SOURCE_DIR}/src/core/src)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/install/test)

# 测试发布模式
add_executable(publish src/publish.cc ${ALL_LIBRIRY_SRCS})
target_link_libraries(publish rt)

# 测试订阅模式
add_executable(subscribe src/subscribe.cc ${ALL_LIBRIRY_SRCS})
target_link_libraries(subscribe rt)
