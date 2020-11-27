cmake_minimum_required(VERSION 3.18)

project(ORB_SLAM3_Mono_Inertial)

add_executable(mono_inertial_euroc mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc PRIVATE ORB_SLAM3::ORB_SLAM3)

add_executable(mono_inertial_tum_vi mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi PRIVATE ORB_SLAM3::ORB_SLAM3)