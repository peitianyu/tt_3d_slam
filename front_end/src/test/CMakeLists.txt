file(GLOB_RECURSE ALL_LIBRIRY_SRCS "../core/*.cc")
file(GLOB_RECURSE ALL_LIBRIRY_TESTS "src/*.c*")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG -Wall -std=c++14 -fPIC ")

# 添加头文件
INCLUDE_DIRECTORIES(../core/src)

# 允许测试
add_definitions(-DTEST_ENABLE)

# 执行文件
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/install/test)

add_executable(all_test ${ALL_LIBRIRY_TESTS} ${ALL_LIBRIRY_SRCS})
target_link_libraries(all_test ${LV_MATH_INSTALL_DIR}/lib/liblv_math.so 
                                ${LV_VIZ_INSTALL_DIR}/lib/libviz.so 
                                ${PCL_LIBRARIES}
                                ${OpenCV_LIBS}
                                ${LV_PLATFORM_INSTALL_DIR}/lib/libplatform.so)


