cmake_minimum_required(VERSION 3.16)
project(ORB_SLAM3 CXX)

# Standard settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# Output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# Dependencies
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
find_package(Pangolin REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)  # Atlas save/load
find_package(OpenSSL REQUIRED)                         # MD5 checksum of the vocabulary in saved atlases

# Optimization/Architecture flags (-O3 comes with the Release build type), for every target in the tree: Eigen picks
# its memory alignment from the instruction set, so all objects of the process must be compiled with the same flags
# (mixing them makes an object allocated with one alignment scheme be freed with the other). -march=native ties the
# binaries to the build machine's CPU, so it is opt-in (local builds); MSVC needs /bigobj for the Eigen-heavy code
option(ORBSLAM3_NATIVE "Optimize for the build machine's CPU (-march=native)" OFF)
if(MSVC)
    add_compile_options(/bigobj)
elseif(ORBSLAM3_NATIVE)
    add_compile_options(-march=native)
endif()

# DBoW2 and g2o from Thirdparty folders (Sophus is header-only)
add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/g2o)

# Interface library for common warnings
add_library(orbslam3_warnings INTERFACE)
target_compile_options(orbslam3_warnings INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# Create the library: shared, except on Windows. The sources have no __declspec annotations, and exporting every
# symbol of a DLL exceeds the 65535-export limit (LNK1189: Eigen/g2o/boost template instances), so the Windows
# executables link it statically
if(WIN32)
    set(ORBSLAM3_LIB_TYPE STATIC)
else()
    set(ORBSLAM3_LIB_TYPE SHARED)
endif()
add_library(${PROJECT_NAME} ${ORBSLAM3_LIB_TYPE}
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
)

# Apply warnings
target_link_libraries(${PROJECT_NAME} PRIVATE orbslam3_warnings)

# Set include directories for the target
target_include_directories(${PROJECT_NAME} PUBLIC
   $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
   $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>
   ${DBoW2_INCLUDE_DIRS}
   ${g2o_INCLUDE_DIRS}
   ${PROJECT_SOURCE_DIR}/include/CameraModels
   ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
)

# Link dependencies
target_link_libraries(${PROJECT_NAME}
    PUBLIC ${OpenCV_LIBS}
    PUBLIC Eigen3::Eigen
    PUBLIC yaml-cpp::yaml-cpp
    ${Pangolin_LIBRARIES}
    PUBLIC DBoW2
    PUBLIC g2o
    PUBLIC Boost::serialization
    PUBLIC OpenSSL::Crypto
    PUBLIC Threads::Threads
)

# Example executables
add_executable(vslamlab_orbslam3_mono Examples/Monocular/vslamlab_orbslam3_mono.cpp)
target_link_libraries(vslamlab_orbslam3_mono PRIVATE ${PROJECT_NAME})

add_executable(vslamlab_orbslam3_rgbd Examples/RGB-D/vslamlab_orbslam3_rgbd.cpp)
target_link_libraries(vslamlab_orbslam3_rgbd PRIVATE ${PROJECT_NAME})

add_executable(vslamlab_orbslam3_stereo Examples/Stereo/vslamlab_orbslam3_stereo.cpp)
target_link_libraries(vslamlab_orbslam3_stereo PRIVATE ${PROJECT_NAME})

add_executable(vslamlab_orbslam3_mono_vi Examples/Monocular-Inertial/vslamlab_orbslam3_mono_vi.cpp)
target_link_libraries(vslamlab_orbslam3_mono_vi PRIVATE ${PROJECT_NAME})

add_executable(vslamlab_orbslam3_stereo_vi Examples/Stereo-Inertial/vslamlab_orbslam3_stereo_vi.cpp)
target_link_libraries(vslamlab_orbslam3_stereo_vi PRIVATE ${PROJECT_NAME})

add_executable(vslamlab_orbslam3_rgbd_vi Examples/RGB-D-Inertial/vslamlab_orbslam3_rgbd_vi.cpp)
target_link_libraries(vslamlab_orbslam3_rgbd_vi PRIVATE ${PROJECT_NAME})

# Online entry point on an ARStream relay (github.com/VSLAM-LAB/arstream, conda package libarstream): built only
# when the header-only subscriber is available, so the package builds without it
find_package(arstream QUIET)
if(arstream_FOUND)
    message(STATUS "arstream found: building vslamlab_orbslam3_mono_stream")
    add_executable(vslamlab_orbslam3_mono_stream Examples/Monocular/vslamlab_orbslam3_mono_stream.cpp)
    target_link_libraries(vslamlab_orbslam3_mono_stream PRIVATE ${PROJECT_NAME} arstream::arstream)
    install(TARGETS vslamlab_orbslam3_mono_stream RUNTIME DESTINATION bin)
endif()

# Install the library and the executables (no headers: the conda package only provides the executables, and the
# generic header names would clash with other packages in the environment)
install(TARGETS
        vslamlab_orbslam3_mono vslamlab_orbslam3_rgbd vslamlab_orbslam3_stereo
        vslamlab_orbslam3_mono_vi vslamlab_orbslam3_rgbd_vi vslamlab_orbslam3_stereo_vi
        RUNTIME DESTINATION bin)
if(NOT WIN32)  # on Windows the static library is already linked into the executables
    install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
endif()
