cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM3)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
set(RDK_SOURCE_DIR ${PROJECT_SOURCE_DIR}/../../core)
set(SRCDIR ${RDK_SOURCE_DIR}/grpc/cpp/gen)
MESSAGE("PROJECT SOURCE DIR: " ${PROJECT_SOURCE_DIR})
MESSAGE("GRPC SOURCE DIR: " ${SRCDIR})
# Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.2)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 3.2 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(realsense2)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Atlas.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Viewer.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/CameraModels/Pinhole.cpp
src/CameraModels/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/MLPnPsolver.cpp
src/GeometricTools.cc
src/TwoViewReconstruction.cc
src/Config.cc
src/Settings.cc
include/System.h
include/Tracking.h
include/LocalMapping.h
include/LoopClosing.h
include/ORBextractor.h
include/ORBmatcher.h
include/FrameDrawer.h
include/Converter.h
include/MapPoint.h
include/KeyFrame.h
include/Atlas.h
include/Map.h
include/MapDrawer.h
include/Optimizer.h
include/Frame.h
include/KeyFrameDatabase.h
include/Sim3Solver.h
include/Viewer.h
include/ImuTypes.h
include/G2oTypes.h
include/CameraModels/GeometricCamera.h
include/CameraModels/Pinhole.h
include/CameraModels/KannalaBrandt8.h
include/OptimizableTypes.h
include/MLPnPsolver.h
include/GeometricTools.h
include/TwoViewReconstruction.h
include/SerializationUtils.h
include/Config.h
include/Settings.h)

add_subdirectory(Thirdparty/g2o)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
-lboost_serialization
-lcrypto
)

# If RealSense SDK is found the library is added and its examples compiled
if(realsense2_FOUND)
    include_directories(${PROJECT_NAME}
    ${realsense_INCLUDE_DIR}
    )
    target_link_libraries(${PROJECT_NAME}
    ${realsense2_LIBRARY}
    )
endif()


# Build examples

# RGB-D examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/initialAttempt)
add_executable(rgbd_realsense_test
        initialAttempt/rgbd_realsense_L515.cc)
target_link_libraries(rgbd_realsense_test ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/initialAttempt)
add_executable(no_imu_test
        initialAttempt/rgbd_noimu.cc)
target_link_libraries(no_imu_test ${PROJECT_NAME})

find_package(PkgConfig REQUIRED)

pkg_check_modules(GRPC REQUIRED grpc) 
pkg_check_modules(GRPCPP REQUIRED grpc++)
pkg_check_modules(PROTOBUF REQUIRED protobuf)

# find_package(Protobuf REQUIRED)

# find_package(PkgConfig REQUIRED)
# find_package(grpc REQUIRED) 
# find_package(grpc++ REQUIRED)
# find_package(protobuf REQUIRED)
# include_directories(${SRCDIR})
# include_directories(${GR PC_INCLUDE_DIRS} ${GRPCPP_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS})

set(SOURCES ${SRCDIR}/proto/api/v1/robot.grpc.pb.cc ${SRCDIR}/proto/api/v1/robot.pb.cc)
set(SOURCES ${SOURCES} ${SRCDIR}/proto/api/component/v1/camera.grpc.pb.cc ${SRCDIR}/proto/api/component/v1/camera.pb.cc)
set(SOURCES ${SOURCES} ${SRCDIR}/proto/api/service/v1/metadata.grpc.pb.cc ${SRCDIR}/proto/api/service/v1/metadata.pb.cc)
set(SOURCES ${SOURCES} ${SRCDIR}/proto/api/common/v1/common.grpc.pb.cc ${SRCDIR}/proto/api/common/v1/common.pb.cc)
set(SOURCES ${SOURCES} ${SRCDIR}/google/api/annotations.pb.cc ${SRCDIR}/google/api/httpbody.pb.cc)
set(SOURCES ${SOURCES} ${SRCDIR}/google/api/http.pb.cc)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/initialAttempt)


add_executable(rgbd_fromStream
        initialAttempt/rgbd_fromStream.cc ${SOURCES})
# target_link_libraries(rgbd_fromStream ${PROJECT_NAME})
target_link_libraries(rgbd_fromStream ${PROJECT_NAME} ${GRPC_LIBRARIES} ${GRPCPP_LIBRARIES} ${PROTOBUF_LIBRARIES})
target_include_directories(rgbd_fromStream PUBLIC ${SRCDIR} ${GRPC_INCLUDE_DIRS} ${GRPCPP_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS})

target_compile_options(rgbd_fromStream PUBLIC ${GRPC_CFLAGS_OTHER} ${GRPCPP_CFLAGS_OTHER} ${PROTOBUF_CFLAGS_OTHER})
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

# add_executable(rgbd_tum
#         Examples/RGB-D/rgbd_tum.cc)
# target_link_libraries(rgbd_tum ${PROJECT_NAME})

# if(realsense2_FOUND)
#     add_executable(rgbd_realsense_D435i
#             Examples/RGB-D/rgbd_realsense_D435i.cc)
#     target_link_libraries(rgbd_realsense_D435i ${PROJECT_NAME})
# endif()


# # RGB-D inertial examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D-Inertial)

# if(realsense2_FOUND)
#     add_executable(rgbd_inertial_realsense_D435i
#             Examples/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc)
#     target_link_libraries(rgbd_inertial_realsense_D435i ${PROJECT_NAME})
# endif()

# #Stereo examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

# add_executable(stereo_kitti
#         Examples/Stereo/stereo_kitti.cc)
# target_link_libraries(stereo_kitti ${PROJECT_NAME})

# add_executable(stereo_euroc
#         Examples/Stereo/stereo_euroc.cc)
# target_link_libraries(stereo_euroc ${PROJECT_NAME})

# add_executable(stereo_tum_vi
#         Examples/Stereo/stereo_tum_vi.cc)
# target_link_libraries(stereo_tum_vi ${PROJECT_NAME})

# if(realsense2_FOUND)
#     add_executable(stereo_realsense_t265
#             Examples/Stereo/stereo_realsense_t265.cc)
#     target_link_libraries(stereo_realsense_t265 ${PROJECT_NAME})

#     add_executable(stereo_realsense_D435i
#             Examples/Stereo/stereo_realsense_D435i.cc)
#     target_link_libraries(stereo_realsense_D435i ${PROJECT_NAME})
# endif()

# #Monocular examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

# add_executable(mono_tum
#         Examples/Monocular/mono_tum.cc)
# target_link_libraries(mono_tum ${PROJECT_NAME})

# add_executable(mono_kitti
#         Examples/Monocular/mono_kitti.cc)
# target_link_libraries(mono_kitti ${PROJECT_NAME})

# add_executable(mono_euroc
#         Examples/Monocular/mono_euroc.cc)
# target_link_libraries(mono_euroc ${PROJECT_NAME})

# add_executable(mono_tum_vi
#         Examples/Monocular/mono_tum_vi.cc)
# target_link_libraries(mono_tum_vi ${PROJECT_NAME})

# if(realsense2_FOUND)
#     add_executable(mono_realsense_t265
#             Examples/Monocular/mono_realsense_t265.cc)
#     target_link_libraries(mono_realsense_t265 ${PROJECT_NAME})

#     add_executable(mono_realsense_D435i
#             Examples/Monocular/mono_realsense_D435i.cc)
#     target_link_libraries(mono_realsense_D435i ${PROJECT_NAME})
# endif()

# #Monocular inertial examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular-Inertial)

# add_executable(mono_inertial_euroc
#         Examples/Monocular-Inertial/mono_inertial_euroc.cc)
# target_link_libraries(mono_inertial_euroc ${PROJECT_NAME})

# add_executable(mono_inertial_tum_vi
#         Examples/Monocular-Inertial/mono_inertial_tum_vi.cc)
# target_link_libraries(mono_inertial_tum_vi ${PROJECT_NAME})

# if(realsense2_FOUND)
#     add_executable(mono_inertial_realsense_t265
#             Examples/Monocular-Inertial/mono_inertial_realsense_t265.cc)
#     target_link_libraries(mono_inertial_realsense_t265 ${PROJECT_NAME})

#     add_executable(mono_inertial_realsense_D435i
#             Examples/Monocular-Inertial/mono_inertial_realsense_D435i.cc)
#     target_link_libraries(mono_inertial_realsense_D435i ${PROJECT_NAME})
# endif()

# #Stereo Inertial examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial)

# add_executable(stereo_inertial_euroc
#         Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
# target_link_libraries(stereo_inertial_euroc ${PROJECT_NAME})

# add_executable(stereo_inertial_tum_vi
#         Examples/Stereo-Inertial/stereo_inertial_tum_vi.cc)
# target_link_libraries(stereo_inertial_tum_vi ${PROJECT_NAME})

# if(realsense2_FOUND)
#     add_executable(stereo_inertial_realsense_t265
#             Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc)
#     target_link_libraries(stereo_inertial_realsense_t265 ${PROJECT_NAME})

#     add_executable(stereo_inertial_realsense_D435i
#             Examples/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
#     target_link_libraries(stereo_inertial_realsense_D435i ${PROJECT_NAME})
# endif()

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Calibration)
# if(realsense2_FOUND)
#     add_executable(recorder_realsense_D435i
#             Examples/Calibration/recorder_realsense_D435i.cc)
#     target_link_libraries(recorder_realsense_D435i ${PROJECT_NAME})

#     add_executable(recorder_realsense_T265
#             Examples/Calibration/recorder_realsense_T265.cc)
#     target_link_libraries(recorder_realsense_T265 ${PROJECT_NAME})
# endif()

# #Old examples

# # RGB-D examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/RGB-D)

# add_executable(rgbd_tum_old
#         Examples_old/RGB-D/rgbd_tum.cc)
# target_link_libraries(rgbd_tum_old ${PROJECT_NAME})

# if(realsense2_FOUND)
#     add_executable(rgbd_realsense_D435i_old
#             Examples_old/RGB-D/rgbd_realsense_D435i.cc)
#     target_link_libraries(rgbd_realsense_D435i_old ${PROJECT_NAME})
# endif()


# # RGB-D inertial examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/RGB-D-Inertial)

# if(realsense2_FOUND)
#     add_executable(rgbd_inertial_realsense_D435i_old
#             Examples_old/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc)
#     target_link_libraries(rgbd_inertial_realsense_D435i_old ${PROJECT_NAME})
# endif()

# #Stereo examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Stereo)

# add_executable(stereo_kitti_old
#         Examples_old/Stereo/stereo_kitti.cc)
# target_link_libraries(stereo_kitti_old ${PROJECT_NAME})

# add_executable(stereo_euroc_old
#         Examples_old/Stereo/stereo_euroc.cc)
# target_link_libraries(stereo_euroc_old ${PROJECT_NAME})

# add_executable(stereo_tum_vi_old
#         Examples_old/Stereo/stereo_tum_vi.cc)
# target_link_libraries(stereo_tum_vi_old ${PROJECT_NAME})

# if(realsense2_FOUND)
#     add_executable(stereo_realsense_t265_old
#             Examples_old/Stereo/stereo_realsense_t265.cc)
#     target_link_libraries(stereo_realsense_t265_old ${PROJECT_NAME})

#     add_executable(stereo_realsense_D435i_old
#             Examples_old/Stereo/stereo_realsense_D435i.cc)
#     target_link_libraries(stereo_realsense_D435i_old ${PROJECT_NAME})
# endif()

# #Monocular examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Monocular)

# add_executable(mono_tum_old
#         Examples_old/Monocular/mono_tum.cc)
# target_link_libraries(mono_tum_old ${PROJECT_NAME})

# add_executable(mono_kitti_old
#         Examples_old/Monocular/mono_kitti.cc)
# target_link_libraries(mono_kitti_old ${PROJECT_NAME})

# add_executable(mono_euroc_old
#         Examples_old/Monocular/mono_euroc.cc)
# target_link_libraries(mono_euroc_old ${PROJECT_NAME})

# add_executable(mono_tum_vi_old
#         Examples_old/Monocular/mono_tum_vi.cc)
# target_link_libraries(mono_tum_vi_old ${PROJECT_NAME})

# if(realsense2_FOUND)
#     add_executable(mono_realsense_t265_old
#             Examples_old/Monocular/mono_realsense_t265.cc)
#     target_link_libraries(mono_realsense_t265_old ${PROJECT_NAME})

#     add_executable(mono_realsense_D435i_old
#             Examples_old/Monocular/mono_realsense_D435i.cc)
#     target_link_libraries(mono_realsense_D435i_old ${PROJECT_NAME})
# endif()

# #Monocular inertial examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Monocular-Inertial)

# add_executable(mono_inertial_euroc_old
#         Examples_old/Monocular-Inertial/mono_inertial_euroc.cc)
# target_link_libraries(mono_inertial_euroc_old ${PROJECT_NAME})

# add_executable(mono_inertial_tum_vi_old
#         Examples_old/Monocular-Inertial/mono_inertial_tum_vi.cc)
# target_link_libraries(mono_inertial_tum_vi_old ${PROJECT_NAME})

# if(realsense2_FOUND)
#     add_executable(mono_inertial_realsense_t265_old
#             Examples_old/Monocular-Inertial/mono_inertial_realsense_t265.cc)
#     target_link_libraries(mono_inertial_realsense_t265_old ${PROJECT_NAME})

#     add_executable(mono_inertial_realsense_D435i_old
#             Examples_old/Monocular-Inertial/mono_inertial_realsense_D435i.cc)
#     target_link_libraries(mono_inertial_realsense_D435i_old ${PROJECT_NAME})
# endif()

# #Stereo Inertial examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Stereo-Inertial)

# add_executable(stereo_inertial_euroc_old
#         Examples_old/Stereo-Inertial/stereo_inertial_euroc.cc)
# target_link_libraries(stereo_inertial_euroc_old ${PROJECT_NAME})

# add_executable(stereo_inertial_tum_vi_old
#         Examples_old/Stereo-Inertial/stereo_inertial_tum_vi.cc)
# target_link_libraries(stereo_inertial_tum_vi_old ${PROJECT_NAME})

# if(realsense2_FOUND)
#     add_executable(stereo_inertial_realsense_t265_old
#             Examples_old/Stereo-Inertial/stereo_inertial_realsense_t265.cc)
#     target_link_libraries(stereo_inertial_realsense_t265_old ${PROJECT_NAME})

#     add_executable(stereo_inertial_realsense_D435i_old
#             Examples_old/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
#     target_link_libraries(stereo_inertial_realsense_D435i_old ${PROJECT_NAME})
# endif()
