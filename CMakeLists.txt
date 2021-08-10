cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM3)

# Build type (default to Release)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

# Debug postfix
set(CMAKE_DEBUG_POSTFIX "_d")

# Check C++11 support
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)
cmake_policy(SET CMP0057 NEW) # enable IN_LIST operator
if(cxx_std_11 IN_LIST CMAKE_CXX_COMPILE_FEATURES)
  add_definitions(-DCOMPILEDWITHC11)
endif()

# Compiler specific options for gcc
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
  # set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-deprecated -O3 -march=native ")
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated -O3 -march=native")
endif()
# Compiler specific options for MSVC
if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3 /bigobj")
  # use the static CRT if other libraries are compiled with /MT
  # (CMake sets /MD by default, but for example Pangolin uses /MT by default)
  #set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
  #set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
endif()

# Output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
if ("${CMAKE_GENERATOR}" MATCHES "Visual Studio")
  # we don't support the Debug and Release subdirs
  foreach (config ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER "${config}" config_upper)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${config_upper} "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${config_upper} "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${config_upper} "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
  endforeach ()
endif ()



# Target definition
set(ORB_SLAM3_LIB_TYPE SHARED)
IF(WIN32)
  # uncomment only one of the lines below:
  # because there is no DLL API in the code,
  # we either have to build STATIC libraries,
  set(ORB_SLAM3_LIB_TYPE STATIC)
  # or we have to export all symbols from all SHARED libraries
  #set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  # Note that the ideal solution would be to add an export API to the relevant classes.
ENDIF(WIN32)

add_library(${PROJECT_NAME} ${ORB_SLAM3_LIB_TYPE}
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
  src/Initializer.cc
  src/Viewer.cc
  src/ImuTypes.cc
  src/G2oTypes.cc
  src/CameraModels/Pinhole.cpp
  src/CameraModels/KannalaBrandt8.cpp
  src/OptimizableTypes.cpp
  src/MLPnPsolver.cpp
  src/TwoViewReconstruction.cc
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
  include/Initializer.h
  include/Viewer.h
  include/ImuTypes.h
  include/G2oTypes.h
  include/CameraModels/GeometricCamera.h
  include/CameraModels/Pinhole.h
  include/CameraModels/KannalaBrandt8.h
  include/OptimizableTypes.h
  include/MLPnPsolver.h
  include/TwoViewReconstruction.h
  include/Config.h
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/CameraModels
)



# Dependencies

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

add_subdirectory(Thirdparty/DBoW2)
target_link_libraries(${PROJECT_NAME} PRIVATE DBoW2)

add_subdirectory(Thirdparty/g2o)
target_link_libraries(${PROJECT_NAME} PRIVATE g2o)


find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3.0 QUIET)
  if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 3.0 not found.")
  endif()
endif()
message(STATUS "OpenCV VERSION: ${OpenCV_VERSION}")
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})

find_package(Eigen3 3.1.0 REQUIRED)
#target_include_directories(${PROJECT_NAME} PRIVATE ${EIGEN3_INCLUDE_DIR})
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "Eigen3_VERSION: ${Eigen3_VERSION}")
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

find_package(Pangolin REQUIRED)
#target_include_directories(${PROJECT_NAME} PRIVATE ${Pangolin_INCLUDE_DIRS})
message(STATUS "Pangolin_LIBRARIES: ${Pangolin_LIBRARIES}")
message(STATUS "Pangolin_VERSION: ${Pangolin_VERSION}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${Pangolin_LIBRARIES})

find_package(realsense2)
if(realsense2_FOUND)
  message(STATUS "realsense2_VERSION: ${realsense2_VERSION}")
  target_link_libraries(${PROJECT_NAME} PRIVATE ${realsense2_LIBRARY})
endif()

find_package(Boost REQUIRED COMPONENTS serialization)
message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")
target_link_libraries(${PROJECT_NAME} PRIVATE ${Boost_LIBRARIES})

if(CMAKE_BUILD_TYPE MATCHES Debug)
  find_package(Backward)
  message(STATUS "BACKWARD_LIBRARIES: ${BACKWARD_LIBRARIES}")
  target_link_libraries(${PROJECT_NAME} PUBLIC Backward::Backward)
endif()

# TODO: OpenSSL is used only in System.cc #include <openssl/md5.h>, but then MD5 is never used.
#set(OPENSSL_USE_STATIC_LIBS TRUE)
#find_package(OpenSSL) # for crypto library
#if(OpenSSL_FOUND)
#  target_include_directories(${PROJECT_NAME} ${OPENSSL_INCLUDE_DIR})
#  target_link_libraries(${PROJECT_NAME} ${OPENSSL_CRYPTO_LIBRARY})
#endif()


### Build examples

# RGB-D examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum PRIVATE ${PROJECT_NAME})


# Stereo examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
  Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti PRIVATE ${PROJECT_NAME})

add_executable(stereo_euroc
  Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc PRIVATE ${PROJECT_NAME})

add_executable(stereo_tum_vi
  Examples/Stereo/stereo_tum_vi.cc)
target_link_libraries(stereo_tum_vi PRIVATE ${PROJECT_NAME})

# Monocular examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum
  Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum PRIVATE ${PROJECT_NAME})

add_executable(mono_kitti
  Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti PRIVATE ${PROJECT_NAME})

add_executable(mono_euroc
  Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc PRIVATE ${PROJECT_NAME})

add_executable(mono_tum_vi
  Examples/Monocular/mono_tum_vi.cc)
target_link_libraries(mono_tum_vi PRIVATE ${PROJECT_NAME})

# Monocular-Inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular-Inertial)

add_executable(mono_inertial_euroc
  Examples/Monocular-Inertial/mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc PRIVATE ${PROJECT_NAME})

add_executable(mono_inertial_tum_vi
  Examples/Monocular-Inertial/mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi PRIVATE ${PROJECT_NAME})

# Stereo-Inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial)

add_executable(stereo_inertial_euroc
  Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
target_link_libraries(stereo_inertial_euroc PRIVATE ${PROJECT_NAME})

add_executable(stereo_inertial_tum_vi
  Examples/Stereo-Inertial/stereo_inertial_tum_vi.cc)
target_link_libraries(stereo_inertial_tum_vi PRIVATE ${PROJECT_NAME})
