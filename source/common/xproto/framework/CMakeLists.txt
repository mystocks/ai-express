cmake_minimum_required(VERSION 2.8)

# 项目名称
project(xproto)

# 添加头文件路径
include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/third_party/include
)
link_directories(
        ${PROJECT_SOURCE_DIR}/third_party/lib
)

# 头文件路径信息
file(GLOB PROJECT_INC
        "include/*.h"
        "include/*.hpp"
        "include/xproto/*.h"
        "include/xproto/utils/*.h"
        "include/xproto/message/protobuf/*.h"
        "include/xproto/manager/*.h"
        "include/xproto/threads/*.h"
        "include/xproto/message/pluginflow/*.h"
        "include/xproto/plugin/*.h"
        )

# 源文件路径信息
file(GLOB PROJECT_SRC
        "src/xproto/*.cpp"
        "src/utils/*.cpp"
        "src/message/protobuf/*.cc"
        "src/threads/*.cpp"
        "src/plugin/*.cpp"
        "src/message/pluginflow/*.cpp"
        )

set(SOURCE_FILES
        ${PROJECT_SRC}
        ${PROJECT_INC}
        )

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} libhobotlog.a)

# 声明全局头文件目录
target_include_directories(${PROJECT_NAME} 
        PUBLIC ${PROJECT_SOURCE_DIR}/include
)

add_subdirectory(test)
add_subdirectory(sample)

# make install才会安装
set(OUTPUT_ROOT ${PROJECT_SOURCE_DIR}/output)
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/xproto)
install(DIRECTORY
        ${CMAKE_SOURCE_DIR}/build/lib/
        DESTINATION ${MY_OUTPUT_ROOT}/lib/)
install(DIRECTORY
        ${PROJECT_SOURCE_DIR}/include/xproto/
        DESTINATION ${MY_OUTPUT_ROOT}/include/xproto)
