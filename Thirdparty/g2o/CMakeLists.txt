CMAKE_MINIMUM_REQUIRED(VERSION 3.18)
# SET(CMAKE_LEGACY_CYGWIN_WIN32 0)

PROJECT(g2o VERSION 0.0.1)

# SET(g2o_C_FLAGS)
# SET(g2o_CXX_FLAGS)

# default built type
# IF(NOT CMAKE_BUILD_TYPE)
#   SET(CMAKE_BUILD_TYPE Release)
# ENDIF()

# MESSAGE(STATUS "BUILD TYPE:" ${CMAKE_BUILD_TYPE})

# SET (G2O_LIB_TYPE SHARED)

# There seems to be an issue with MSVC8
# see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=83
# if(MSVC90)
#   add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
#   message(STATUS "Disabling memory alignment for MSVC8")
# endif(MSVC90)

# Set the output directory for the build executables and libraries
# IF(WIN32)
#   SET(g2o_LIBRARY_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/bin CACHE PATH "Target for the libraries")
# ELSE(WIN32)
#   SET(g2o_LIBRARY_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/lib CACHE PATH "Target for the libraries")
# ENDIF(WIN32)
# SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
# SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
# SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${g2o_RUNTIME_OUTPUT_DIRECTORY})

# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse, QGLViewer, and Eigen3.
# LIST(APPEND CMAKE_MODULE_PATH ${g2o_SOURCE_DIR}/cmake_modules)

# Detect OS and define macros appropriately
# IF(UNIX)
#   ADD_DEFINITIONS(-DUNIX)
#   MESSAGE(STATUS "Compiling on Unix")
# ENDIF(UNIX)

# Eigen library parallelise itself, though, presumably due to performance issues
# OPENMP is experimental. We experienced some slowdown with it
#find_package(OpenMP)
#set(G2O_USE_OPENMP OFF CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
#if(OPENMP_FOUND AND G2O_USE_OPENMP)
  # set (G2O_OPENMP 1)
  # set(g2o_C_FLAGS "${g2o_C_FLAGS} ${OpenMP_C_FLAGS}")
  # set(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
  # message(STATUS "Compiling with OpenMP support")
#endif(OPENMP_FOUND AND G2O_USE_OPENMP)

# Compiler specific options for gcc
# SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
# SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=native")
# SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
# SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")

# activate warnings !!!
# SET(g2o_C_FLAGS "${g2o_C_FLAGS} -Wall -W")
# SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Wall -W")

# specifying compiler flags
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${g2o_CXX_FLAGS}")
# SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${g2o_C_FLAGS}")

# Find Eigen3
# SET(EIGEN3_INCLUDE_DIR ${G2O_EIGEN3_INCLUDE})
find_package(Eigen3 CONFIG REQUIRED)
# IF(EIGEN3_FOUND)
#   SET(G2O_EIGEN3_INCLUDE ${EIGEN3_INCLUDE_DIR} CACHE PATH "Directory of Eigen3")
# ELSE(EIGEN3_FOUND)
#   SET(G2O_EIGEN3_INCLUDE "" CACHE PATH "Directory of Eigen3")
# ENDIF(EIGEN3_FOUND)

# Generate config.h
# SET(G2O_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")
# configure_file(config.h.in ${g2o_SOURCE_DIR}/config.h)

# Set up the top-level include directories
# INCLUDE_DIRECTORIES(
# ${g2o_SOURCE_DIR}/core
# ${g2o_SOURCE_DIR}/types
# ${g2o_SOURCE_DIR}/stuff 
# ${G2O_EIGEN3_INCLUDE})

# Include the subdirectories
add_library(${PROJECT_NAME} STATIC)

add_library(${CMAKE_PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_sources(${PROJECT_NAME}
               PRIVATE
               g2o/types/types_sba.cpp
               g2o/types/types_six_dof_expmap.cpp
               g2o/types/types_seven_dof_expmap.cpp
               g2o/core/hyper_graph_action.cpp
               g2o/core/hyper_graph.cpp
               g2o/core/marginal_covariance_cholesky.cpp
               g2o/core/matrix_structure.cpp
               g2o/core/batch_stats.cpp
               g2o/core/parameter.cpp
               g2o/core/cache.cpp
               g2o/core/optimizable_graph.cpp
               g2o/core/solver.cpp
               g2o/core/optimization_algorithm_factory.cpp
               g2o/core/estimate_propagator.cpp
               g2o/core/factory.cpp
               g2o/core/sparse_optimizer.cpp
               g2o/core/hyper_dijkstra.cpp
               g2o/core/parameter_container.cpp
               g2o/core/optimization_algorithm.cpp
               g2o/core/optimization_algorithm_with_hessian.cpp
               g2o/core/optimization_algorithm_levenberg.cpp
               g2o/core/optimization_algorithm_gauss_newton.cpp
               g2o/core/jacobian_workspace.cpp
               g2o/core/robust_kernel.cpp
               g2o/core/robust_kernel_factory.cpp
               g2o/core/robust_kernel_impl.cpp
               g2o/stuff/os_specific.c
               g2o/stuff/timeutil.cpp
               g2o/stuff/string_tools.cpp
               g2o/stuff/property.cpp)

target_include_directories(${PROJECT_NAME}
                           PRIVATE
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../>
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/g2o/types>
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/g2o/core>
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/g2o/stuff>
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/g2o/solvers>
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/g2o>
                           $<INSTALL_INTERFACE:include/>
                           $<INSTALL_INTERFACE:include/Thirdparty/g2o/types>
                           $<INSTALL_INTERFACE:include/Thirdparty/g2o/core>
                           $<INSTALL_INTERFACE:include/Thirdparty/g2o/stuff>
                           $<INSTALL_INTERFACE:include/Thirdparty/g2o/solvers>
                           $<INSTALL_INTERFACE:include/Thirdparty/g2o>)

target_link_libraries(${PROJECT_NAME}
                      PUBLIC
                      Eigen3::Eigen)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

if(WIN32)
  target_compile_definitions(${PROJECT_NAME} PRIVATE WINDOWS COMPILEDWITHC11)
  message(STATUS "Compiling g2o on Windows")
elseif(CYGWIN)
  message(STATUS "Compiling g2o on Cygwin")
  target_compile_definitions(${PROJECT_NAME} PRIVATE CYGWIN)
elseif(APPLE)
  target_compile_definitions(${PROJECT_NAME} PRIVATE UNIX)
  message(STATUS "Compiling g2o on OSX")
elseif(UNIX)
  target_compile_definitions(${PROJECT_NAME} PRIVATE UNIX)
  message(STATUS "Compiling g2o on Unix")
endif(WIN32)

# detect Android Cross Compiler
# based on android-cmake which sets the variable ANDROID for us
if(ANDROID)
  target_compile_definitions(${PROJECT_NAME} PRIVATE ANDROID)
  message(STATUS "Cross compiling g2o for Android")
endif()

install(DIRECTORY g2o/types/
        DESTINATION include/ThirdParty/g2o/types/
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY g2o/core/
        DESTINATION include/ThirdParty/g2o/core/
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY g2o/solvers/
        DESTINATION include/ThirdParty/g2o/solvers/
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY g2o/stuff/
        DESTINATION include/ThirdParty/g2o/stuff/
        FILES_MATCHING PATTERN "*.h")