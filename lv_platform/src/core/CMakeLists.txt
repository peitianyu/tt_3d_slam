file(GLOB_RECURSE ALL_LIBRIRY_HEDS "src/*.h*")
file(GLOB_RECURSE ALL_LIBRIRY_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc)

# 添加头文件
INCLUDE_DIRECTORIES(src ${LV_MATH_INSTALL_DIR}/include/math)

# 添加动态库
add_library(${PROJECT_NAME} SHARED ${ALL_LIBRIRY_SRCS})
target_link_libraries(${PROJECT_NAME} rt ${LV_MATH_INSTALL_DIR}/lib/liblv_math.so)

# 安装到本工程的lib文件夹下
set(INSTALL_LIB ${PROJECT_SOURCE_DIR}/install/lib)
set(INSTALL_INCLUDE ${PROJECT_SOURCE_DIR}/install/include)

# 安装platform库
install(TARGETS ${PROJECT_NAME} DESTINATION ${INSTALL_LIB})

# 安装头文件
install(DIRECTORY  ./ DESTINATION ${INSTALL_INCLUDE} FILES_MATCHING PATTERN "*.h*")