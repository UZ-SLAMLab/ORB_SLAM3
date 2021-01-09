cmake_minimum_required(VERSION 3.18)

project(ORB_SLAM3_Mono)

add_executable(mono_tum mono_tum.cc)
target_link_libraries(mono_tum PRIVATE ORB_SLAM::ORB_SLAM3)

add_executable(mono_kitti mono_kitti.cc)
target_link_libraries(mono_kitti PRIVATE ORB_SLAM::ORB_SLAM3)

add_executable(mono_euroc mono_euroc.cc)
target_link_libraries(mono_euroc PRIVATE ORB_SLAM::ORB_SLAM3)

add_executable(mono_tum_vi mono_tum_vi.cc)
target_link_libraries(mono_tum_vi PRIVATE ORB_SLAM::ORB_SLAM3)
