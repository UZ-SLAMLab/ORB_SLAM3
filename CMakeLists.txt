cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM3)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#Next dependencies could be required (OpenCV, Eigen3, Pangolin, Boost, openssl, OpenGL, GLEW)

find_package(OpenCV 4.7)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

find_package(Boost 1.82.0 REQUIRED serialization)
find_package(openssl REQUIRED QUIET)

find_package(OpenGL REQUIRED QUIET)
find_package(GLEW REQUIRED QUIET)

set(OPENGL_INCLUDE /opt/homebrew/include)
set(OPENSSL_LIB_INCLUDE /opt/homebrew/opt/openssl/include)
set(BOOST_LIB_INCLUDE /opt/homebrew/opt/boost/include)

#Pangolin include path should be changed, it is for example here:
SET( Pangolin_LIB_INCLUDE
/Users/serg/PycharmProjects/Pangolin/components/pango_core/include;
/Users/serg/PycharmProjects/Pangolin/components/pango_display/include;
/Users/serg/PycharmProjects/Pangolin/components/pango_geometry/include;
/Users/serg/PycharmProjects/Pangolin/components/pango_glgeometry/include;
/Users/serg/PycharmProjects/Pangolin/components/pango_image/include;
/Users/serg/PycharmProjects/Pangolin/components/pango_opengl/include;
/Users/serg/PycharmProjects/Pangolin/components/pango_packetstream/include;
/Users/serg/PycharmProjects/Pangolin/components/pango_plot/include;
/Users/serg/PycharmProjects/Pangolin/components/pango_python/include;
/Users/serg/PycharmProjects/Pangolin/components/pango_scene/include;
/Users/serg/PycharmProjects/Pangolin/components/pango_tools/include;
/Users/serg/PycharmProjects/Pangolin/components/pango_vars/include;
/Users/serg/PycharmProjects/Pangolin/components/pango_video/include;
/Users/serg/PycharmProjects/Pangolin/components/pango_windowing/include;
/Users/serg/PycharmProjects/Pangolin/components/tinyobj/include )

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
${Pangolin_LIB_INCLUDE}
${OPENSSL_LIB_INCLUDE}
${BOOST_LIB_INCLUDE}
${OPENGL_INCLUDE}
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
src/md5.cc
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
include/Settings.h
include/md5.h
)

add_subdirectory(Thirdparty/g2o)

set(BOOST_LIBS /opt/homebrew/opt/boost/lib)
set(OPENSSL_LIBS  /opt/homebrew/opt/openssl/lib)

target_link_libraries(${PROJECT_NAME}
PRIVATE Boost::serialization
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.dylib
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.dylib
${OPENSSL_LIBS}
)

# Build examples

#Monocular examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum Examples/Monocular/mono_tum.cc)
target_compile_definitions(mono_tum PUBLIC HAVE_EIGEN HAVE_GLEW)
target_link_libraries(mono_tum PUBLIC ${GLEW_LIBRARY} ${OPENGL_LIBRARIES} ${OPENSSL_LIBS} ${OpenCV_LIBS} ${PROJECT_NAME})

add_executable(mono_kitti Examples/Monocular/mono_kitti.cc)
target_compile_definitions(mono_kitti PUBLIC HAVE_EIGEN HAVE_GLEW)
target_link_libraries(mono_kitti PUBLIC ${GLEW_LIBRARY} ${OPENGL_LIBRARIES} ${OPENSSL_LIBS} ${OpenCV_LIBS} ${PROJECT_NAME})

add_executable(mono_euroc Examples/Monocular/mono_euroc.cc)
target_compile_definitions(mono_euroc PUBLIC HAVE_EIGEN HAVE_GLEW)
target_link_libraries(mono_euroc PUBLIC ${GLEW_LIBRARY} ${OPENGL_LIBRARIES} ${OPENSSL_LIBS} ${OpenCV_LIBS} ${PROJECT_NAME})

add_executable(mono_tum_vi Examples/Monocular/mono_tum_vi.cc)
target_compile_definitions(mono_tum_vi PUBLIC HAVE_EIGEN HAVE_GLEW)
target_link_libraries(mono_tum_vi PUBLIC ${GLEW_LIBRARY} ${OPENGL_LIBRARIES} ${OPENSSL_LIBS} ${OpenCV_LIBS} ${PROJECT_NAME})

#Stereo example
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_tum_vi Examples/Stereo/stereo_tum_vi.cc)
target_compile_definitions(stereo_tum_vi PUBLIC HAVE_EIGEN HAVE_GLEW)
target_link_libraries(stereo_tum_vi PUBLIC ${GLEW_LIBRARY} ${OPENGL_LIBRARIES} ${OPENSSL_LIBS} ${OpenCV_LIBS} ${PROJECT_NAME})

#Stereo Inertial example
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial)

add_executable(stereo_inertial_tum_vi Examples/Stereo-Inertial/stereo_inertial_tum_vi.cc)
target_compile_definitions(stereo_inertial_tum_vi PUBLIC HAVE_EIGEN HAVE_GLEW)
target_link_libraries(stereo_inertial_tum_vi PUBLIC ${GLEW_LIBRARY} ${OPENGL_LIBRARIES} ${OPENSSL_LIBS} ${OpenCV_LIBS} ${PROJECT_NAME})
