## 
# the compile script for loongson car
# use make to compile the out file
# use clean to clear last built
##
cmake_minimum_required(VERSION 3.16)    # 限制最低CMake版本

###只修改这一部分

# 设置工程目录
set(PROJECT_DIR "~/loongson/ABI1/cvpass")
# 设置交叉编译工具路径 --> (需根据自己实际路径进行修改)
set(CROSS_COMPILE loongarch64-linux-gnu-)
# 设置交叉编译工具路径 --> (需根据自己实际路径进行修改)
set(CROSS_COMPILER_DIR /opt/loongarch64-gnu-gcc-ABI1)

###修改部分结束

# 指定交叉编译工具所使用的C语言编译器
set(CMAKE_C_COMPILER ${CROSS_COMPILER_DIR}/bin/${CROSS_COMPILE}gcc)
# 指定交叉编译工具所使用的C++编译器
set(CMAKE_CXX_COMPILER ${CROSS_COMPILER_DIR}/bin/${CROSS_COMPILE}g++)

# 项目名称
project(cvpass)

message(STATUS ${PROJECT_DIR})

# 指定所用的源文件路径
aux_source_directory(${PROJECT_SOURCE_DIR}/ SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/IO/ SRC)

#指定IO头文件路径
include_directories(${PROJECT_DIR}/IO/inc/)
# 指定OpenCV所用头文件路径
include_directories(${PROJECT_DIR}/opencv/include/opencv4/opencv2/)
include_directories(${PROJECT_DIR}/opencv/include/opencv4/)
include_directories(${PROJECT_DIR}/opencv/include)
# 找到opencv所需的库文件并保存到变量中
find_library(OPENCV_CORE      opencv_core      ${PROJECT_DIR}/opencv/lib/)
find_library(OPENCV_IMGPROC   opencv_imgproc   ${PROJECT_DIR}/opencv/lib/)
find_library(OPENCV_HIGHGUI   opencv_highgui   ${PROJECT_DIR}/opencv/lib/)
find_library(OPENCV_VIDEOIO   opencv_videoio   ${PROJECT_DIR}/opencv/lib/)
find_library(OPENCV_IMGCODECS opencv_imgcodecs ${PROJECT_DIR}/opencv/lib/)

# 若需使用静态库，可带上下面两行程序，具体可参考龙邱B站视频教程
# link_directories(${PROJECT_SOURCE_DIR}/a)
# link_libraries(ss)
set(DCMAKE_SKIP_RPATH TRUE)
set(CMAKE_SKIP_BUILD_RPATH TRUE)

find_package(Threads REQUIRED)


# 由源文件生成一个可执行程序
add_executable(main ${SRC})

# 连接OpenCV动态库和线程库
target_link_libraries(main PUBLIC ${OPENCV_CORE} ${OPENCV_IMGPROC} ${OPENCV_HIGHGUI} ${OPENCV_VIDEOIO} ${OPENCV_IMGCODECS} ${CMAKE_THREAD_LIBS_INIT})

# 连接多线程pthread库
#target_link_libraries(main PUBLIC pthread)
