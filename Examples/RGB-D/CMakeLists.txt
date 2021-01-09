cmake_minimum_required(VERSION 3.18)

project(ORB_SLAM3_RGBD)

add_executable(rgbd_tum rgbd_tum.cc)
target_link_libraries(rgbd_tum PRIVATE ORB_SLAM::ORB_SLAM3)
