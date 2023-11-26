cmake_minimum_required(VERSION 3.5)
project(orb_slam3_ros2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# add_definitions(-Wno-deprecated-declarations)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
   ${PROJECT_SOURCE_DIR}
   ${PROJECT_SOURCE_DIR}/../../../
   ${PROJECT_SOURCE_DIR}/../../../include
   ${PROJECT_SOURCE_DIR}/../../../include/CameraModels
   ${PROJECT_SOURCE_DIR}/../../../Thirdparty/Sophus
   ${Pangolin_INCLUDE_DIRS}
)

set(LIBS 
   ${OpenCV_LIBS} 
   ${EIGEN3_LIBS}
   ${Pangolin_LIBRARIES}
   ${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
   ${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
   ${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM3.so
   -lboost_system
)

add_executable(mono_inertial
   src/mono_inertial_node.cpp
   src/ros_mono_inertial.cc
)

ament_target_dependencies(mono_inertial
   rclcpp
   sensor_msgs
   cv_bridge
)

target_link_libraries(mono_inertial
   ${LIBS}
)


install(TARGETS
   mono_inertial
   DESTINATION lib/${PROJECT_NAME}
)

ament_package()
