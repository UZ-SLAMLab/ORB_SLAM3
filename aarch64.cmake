set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_SYSROOT /home/zhangqx/orb-slam/sysroots/ZCU104)
# set(CMAKE_SYSROOT /home/zhangqx/orb-slam/sysroots/ZCU104_v2.7)
set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc-10)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++-10)

set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# set(OpenCV_DIR ${CMAKE_SYSROOT}/opt/opencv-4.4.0/lib/cmake/opencv4)
set(Pangolin_DIR ${CMAKE_SYSROOT}/opt/Pangolin/lib/cmake/Pangolin)
# set(Pangolin_DIR /home/zhangqx/orb-slam/Pangolin-0.7/build)

