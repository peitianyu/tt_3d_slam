file(GLOB_RECURSE ALL_LIBRIRY_SRCS ${PROJECT_SOURCE_DIR}/src/core/viz/*.cc)

# 添加.h文件
include_directories(${PROJECT_SOURCE_DIR}/src/core
                    ${LV_MATH_INSTALL_DIR}/include/math)

# 允许测试
add_definitions(-DTEST_ENABLE)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/install/test)

# 测试模块
add_executable(viz src/test_visual.cc ${ALL_LIBRIRY_SRCS})
target_link_libraries(viz ${LV_MATH_INSTALL_DIR}/lib/liblv_math.so ${PCL_LIBRARIES})
