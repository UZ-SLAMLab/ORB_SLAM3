cmake_minimum_required(VERSION 3.18)

project(ORB_SLAM3_Stereo)

add_executable(stereo_kitti stereo_kitti.cc)
target_link_libraries(stereo_kitti PRIVATE ORB_SLAM::ORB_SLAM3)

add_executable(stereo_euroc stereo_euroc.cc)
target_link_libraries(stereo_euroc PRIVATE ORB_SLAM::ORB_SLAM3)

add_executable(stereo_tum_vi stereo_tum_vi.cc)
target_link_libraries(stereo_tum_vi PRIVATE ORB_SLAM::ORB_SLAM3)
