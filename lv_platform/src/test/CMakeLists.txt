file(GLOB_RECURSE ALL_LIBRIRY_SRCS ${PROJECT_SOURCE_DIR}/src/core/src/*.cc)

# 添加.h文件
include_directories(${PROJECT_SOURCE_DIR}/src/core/src
                    ${LV_MATH_INSTALL_DIR}/include/math)

# 允许测试
add_definitions(-DTEST_ENABLE)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/install/test)

# 测试模块
add_executable(all_test src/test.cc ${ALL_LIBRIRY_SRCS})
target_link_libraries(all_test rt ${LV_MATH_INSTALL_DIR}/lib/liblv_math.so)

# 测试发布模式
add_executable(publish src/publish.cc ${ALL_LIBRIRY_SRCS})
target_link_libraries(publish rt ${LV_MATH_INSTALL_DIR}/lib/liblv_math.so)

# 测试订阅模式
add_executable(subscribe src/subscribe.cc ${ALL_LIBRIRY_SRCS})
target_link_libraries(subscribe rt ${LV_MATH_INSTALL_DIR}/lib/liblv_math.so)