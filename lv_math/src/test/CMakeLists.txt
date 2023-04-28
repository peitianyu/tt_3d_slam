file(GLOB_RECURSE ALL_LIBRIRY_SRCS "../core/math/*.c*")

# 添加头文件
INCLUDE_DIRECTORIES(../core/math)

# 运行测试
add_definitions(-DTEST_ENABLE)

# 执行文件
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/install/test)

# 测试文件
add_executable(all_tests src/test.cc ${ALL_LIBRIRY_SRCS})






