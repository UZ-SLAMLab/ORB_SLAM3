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

find_package(OpenCV 4)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3.2)
  if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 3.2 not found.")
  endif()
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
-lboost_filesystem
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


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
      add_executable(rec_realsense
         initialAttempt/record_realsense.cc)
      target_link_libraries(rec_realsense ${PROJECT_NAME})
      
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/initialAttempt)
add_executable(mono_file
        initialAttempt/mono_fromFiles.cc)
target_link_libraries(mono_file ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
      add_executable(rgbd_fromFiles
         initialAttempt/rgbd_fromFiles.cc)
      target_link_libraries(rgbd_fromFiles ${PROJECT_NAME})

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/initialAttempt)
# add_executable(no_imu_test
#         initialAttempt/rgbd_noimu.cc)
# target_link_libraries(no_imu_test ${PROJECT_NAME})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(viam_main_v1
        viam-main-dir/main_v1.cc)
target_link_libraries(viam_main_v1 ${PROJECT_NAME})

if(FALSE)#if(EXISTS ${SRCDIR})
   find_package(PkgConfig REQUIRED)

   pkg_check_modules(GRPC REQUIRED grpc) 
   pkg_check_modules(GRPCPP REQUIRED grpc++)
   pkg_check_modules(PROTOBUF REQUIRED protobuf=3.21.4)

   set(SOURCES ${SRCDIR}/proto/api/robot/v1/robot.grpc.pb.cc ${SRCDIR}/proto/api/robot/v1/robot.pb.cc)
   set(SOURCES ${SOURCES} ${SRCDIR}/proto/api/component/camera/v1/camera.grpc.pb.cc ${SRCDIR}/proto/api/component/camera/v1/camera.pb.cc)
   set(SOURCES ${SOURCES} ${SRCDIR}/proto/api/service/metadata/v1/metadata.grpc.pb.cc ${SRCDIR}/proto/api/service/metadata/v1/metadata.pb.cc)
   set(SOURCES ${SOURCES} ${SRCDIR}/proto/api/common/v1/common.grpc.pb.cc ${SRCDIR}/proto/api/common/v1/common.pb.cc)
   set(SOURCES ${SOURCES} ${SRCDIR}/google/api/annotations.pb.cc ${SRCDIR}/google/api/httpbody.pb.cc)
   set(SOURCES ${SOURCES} ${SRCDIR}/google/api/http.pb.cc)
   set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/initialAttempt)


   add_executable(rgbd_fromStream
         initialAttempt/rgbd_fromStream.cc ${SOURCES})
   target_link_libraries(rgbd_fromStream ${PROJECT_NAME} ${GRPC_LIBRARIES} ${GRPCPP_LIBRARIES} ${PROTOBUF_LIBRARIES})
   target_include_directories(rgbd_fromStream PUBLIC ${SRCDIR} ${GRPC_INCLUDE_DIRS} ${GRPCPP_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS})
   target_compile_options(rgbd_fromStream PUBLIC ${GRPC_CFLAGS_OTHER} ${GRPCPP_CFLAGS_OTHER} ${PROTOBUF_CFLAGS_OTHER})
   endif()
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)
