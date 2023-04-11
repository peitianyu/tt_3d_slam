file(GLOB_RECURSE ALL_LIBRIRY_HEDS "src/*.h*")
file(GLOB_RECURSE ALL_LIBRIRY_SRCS "src/*.c*")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG -Wall -std=c++14 -fPIC ")

# 添加头文件
INCLUDE_DIRECTORIES(src)

# 安装到本工程的lib文件夹下
set(INSTALL_LIB ${PROJECT_SOURCE_DIR}/install/lib)
set(INSTALL_INCLUDE ${PROJECT_SOURCE_DIR}/install/include)

# 添加动态库
# add_library(${PROJECT_NAME} SHARED ${ALL_LIBRIRY_SRCS})
# target_link_libraries(${PROJECT_NAME} ${LV_MATH_INSTALL_DIR}/lib/liblv_math.so)

# install(TARGETS ${PROJECT_NAME} DESTINATION ${INSTALL_LIB})
# # 必要的头文件才需要放进来
# install(DIRECTORY  ./ DESTINATION ${INSTALL_INCLUDE} FILES_MATCHING PATTERN "*.h*")
        
