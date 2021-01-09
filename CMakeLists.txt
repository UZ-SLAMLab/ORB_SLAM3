# This cmake is a modified version of https://github.com/forexample/package-example/blob/master/Foo/CMakeLists.txt
cmake_minimum_required(VERSION 3.18)
project(ORB_SLAM3 VERSION 0.3 LANGUAGES CXX)

# Set Libraries default visibility to hidden
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN YES)

# Add CMake Modules folder
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# Handle installation of pdb-files for MSVC
include(HandlePDB)
###############################      Option      #############################################
option(BUILD_EXAMPLES       "Build Examples"    OFF)
option(BUILD_TOOLS          "Build Tools"       OFF)
###############################      Option      #############################################

###############################     Packages     #############################################
find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc imgcodecs) # opencv_core opencv_img_codecs
find_package(Eigen3 CONFIG REQUIRED) # Eigen3::Eigen
find_package(Pangolin CONFIG REQUIRED) # pangolin
find_package(Boost REQUIRED COMPONENTS serialization) # serialization
find_package(OpenSSL REQUIRED) # crypto
# Thirdparty packages
add_subdirectory(Thirdparty/g2o)
add_subdirectory(Thirdparty/DBoW2)
###############################     Packages     #############################################

###############################     ORB-SLAM     #############################################
add_library(${PROJECT_NAME} STATIC)

add_library(ORB_SLAM::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_sources(${PROJECT_NAME}
               PRIVATE
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
               src/PnPsolver.cc
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
               src/TwoViewReconstruction.cc)

target_include_directories(${PROJECT_NAME}
                           PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/CameraModels>
                           ${OpenCV_INCLUDE_DIRS}
                           ${Pangolin_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}
                      PUBLIC
                      Eigen3::Eigen
                      ${OpenCV_LIBRARIES}
                      Boost::serialization
                      OpenSSL::Crypto
                      pangolin
                      # ThirdParty
                      ORB_SLAM::DBoW2
                      ORB_SLAM::g2o)

target_compile_options(${PROJECT_NAME}
                       PRIVATE
                       $<$<CXX_COMPILER_ID:MSVC>:/bigobj>)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

target_compile_definitions(${PROJECT_NAME} PUBLIC COMPILEDWITHC11)
include(CompilerWarnings)
set_project_warnings(${PROJECT_NAME})
###############################     ORB-SLAM     #############################################

if(BUILD_EXAMPLES)
add_subdirectory(Examples)
endif()

if(BUILD_TOOLS)
add_subdirectory(tools)
endif()

###############################      Install      #############################################
include(GNUInstallDirs)

set(config_install_dir "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")

# Configuration
set(version_config "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake")
set(project_config "${generated_dir}/${PROJECT_NAME}Config.cmake")
set(TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
set(namespace "ORB_SLAM::")

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    "${version_config}" COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${PROJECT_NAME}Config.cmake.in"
    "${project_config}"
    INSTALL_DESTINATION "${config_install_dir}"
)

install(
    TARGETS ${PROJECT_NAME} DBoW2 g2o
    EXPORT "${TARGETS_EXPORT_NAME}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

# Headers:
install (DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")

install(
    FILES "${project_config}" "${version_config}"
    DESTINATION "${config_install_dir}"
)

# install pdb-files
if(MSVC)
install(
      # this path is generated from HandlePDB.cmake
      DIRECTORY "${pdb_output_dir}/"
      DESTINATION ${pdb_dst}
)
endif()
install(
    EXPORT "${TARGETS_EXPORT_NAME}"
    NAMESPACE "${namespace}"
    DESTINATION "${config_install_dir}"
)
###############################      Install      #############################################
