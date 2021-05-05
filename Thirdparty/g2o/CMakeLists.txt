cmake_minimum_required(VERSION 2.8)
set(CMAKE_LEGACY_CYGWIN_WIN32 0)
project(g2o)

set(G2O_LIB_TYPE SHARED)
if(WIN32)
  set(G2O_LIB_TYPE STATIC)
endif(WIN32)

set(G2O_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR})

set(G2O_C_FLAGS)
set(G2O_CXX_FLAGS)

# default built type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "BUILD TYPE:" ${CMAKE_BUILD_TYPE})


# There seems to be an issue with MSVC8
# see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=83
if(MSVC90)
  add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
  message(STATUS "Disabling memory alignment for MSVC8")
endif(MSVC90)

# Set the output directory for the build executables and libraries
set(G2O_RUNTIME_OUTPUT_DIRECTORY ${G2O_SOURCE_DIR}/bin CACHE PATH "Target for the binaries")
set(G2O_LIBRARY_OUTPUT_DIRECTORY ${G2O_SOURCE_DIR}/lib CACHE PATH "Target for the libraries")
if(WIN32 AND G2O_LIB_TYPE MATCHES SHARED)
  set(G2O_LIBRARY_OUTPUT_DIRECTORY ${G2O_SOURCE_DIR}/bin CACHE PATH "Target for the libraries")
endif()
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${G2O_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${G2O_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${G2O_RUNTIME_OUTPUT_DIRECTORY})

# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse, QGLViewer, and Eigen3.
list(APPEND CMAKE_MODULE_PATH ${G2O_SOURCE_DIR}/cmake_modules)


# Detect OS and define macros appropriately
if(WIN32)
  add_definitions(-DWINDOWS)
  message(STATUS "Compiling g2o on Windows")
elseif(CYGWIN)
  message(STATUS "Compiling g2o on Cygwin")
  add_definitions(-DCYGWIN)
elseif(APPLE)
  add_definitions(-DUNIX)
  message(STATUS "Compiling g2o on OSX")
elseif(UNIX)
  add_definitions(-DUNIX)
  message(STATUS "Compiling g2o on Unix")
endif(WIN32)

# detect Android Cross Compiler
# based on android-cmake which sets the variable ANDROID for us
if(ANDROID)
  add_definitions(-DANDROID)
  message(STATUS "Cross compiling g2o for Android")
endif()

# Eigen library parallelise itself, though, presumably due to performance issues
# OPENMP is experimental. We experienced some slowdown with it
find_package(OpenMP)
set(G2O_USE_OPENMP OFF CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
if(OPENMP_FOUND AND G2O_USE_OPENMP)
  set(G2O_OPENMP 1)
  set(G2O_C_FLAGS "${G2O_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(G2O_CXX_FLAGS "${G2O_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
  message(STATUS "Compiling with OpenMP support")
endif(OPENMP_FOUND AND G2O_USE_OPENMP)

# Compiler specific options for gcc
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=native")

  # activate warnings
  set(G2O_C_FLAGS "${G2O_C_FLAGS} -Wall -W")
  set(G2O_CXX_FLAGS "${G2O_CXX_FLAGS} -Wall -W")
endif()
# Compiler specific options for MSVC
if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  set(G2O_C_FLAGS "${G2O_C_FLAGS} /W3")
  set(G2O_CXX_FLAGS "${G2O_CXX_FLAGS} /W3")
endif()

# specifying compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${G2O_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${G2O_C_FLAGS}")

# Find Eigen3
set(EIGEN3_INCLUDE_DIR ${G2O_EIGEN3_INCLUDE})
find_package(Eigen3 3.1.0 REQUIRED)
if(EIGEN3_FOUND)
  set(G2O_EIGEN3_INCLUDE ${EIGEN3_INCLUDE_DIR} CACHE PATH "Directory of Eigen3")
else(EIGEN3_FOUND)
  set(G2O_EIGEN3_INCLUDE "" CACHE PATH "Directory of Eigen3")
endif(EIGEN3_FOUND)

# Generate config.h
set(G2O_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")
configure_file(config.h.in ${G2O_SOURCE_DIR}/config.h)

# set up the top-level include directories
include_directories(
  ${G2O_SOURCE_DIR}/core
  ${G2O_SOURCE_DIR}/types
  ${G2O_SOURCE_DIR}/stuff
  ${G2O_EIGEN3_INCLUDE}
)


# Include the subdirectories
add_library(g2o ${G2O_LIB_TYPE}
  #types
  g2o/types/types_sba.h
  g2o/types/types_six_dof_expmap.h
  g2o/types/types_sba.cpp
  g2o/types/types_six_dof_expmap.cpp
  g2o/types/types_seven_dof_expmap.cpp
  g2o/types/types_seven_dof_expmap.h
  g2o/types/se3quat.h
  g2o/types/se3_ops.h
  g2o/types/se3_ops.hpp
  #core
  g2o/core/base_edge.h
  g2o/core/base_binary_edge.h
  g2o/core/hyper_graph_action.cpp
  g2o/core/base_binary_edge.hpp
  g2o/core/hyper_graph_action.h
  g2o/core/base_multi_edge.h
  g2o/core/hyper_graph.cpp
  g2o/core/base_multi_edge.hpp
  g2o/core/hyper_graph.h
  g2o/core/base_unary_edge.h
  g2o/core/linear_solver.h
  g2o/core/base_unary_edge.hpp
  g2o/core/marginal_covariance_cholesky.cpp
  g2o/core/base_vertex.h
  g2o/core/marginal_covariance_cholesky.h
  g2o/core/base_vertex.hpp
  g2o/core/matrix_structure.cpp
  g2o/core/batch_stats.cpp
  g2o/core/matrix_structure.h
  g2o/core/batch_stats.h
  g2o/core/openmp_mutex.h
  g2o/core/block_solver.h
  g2o/core/block_solver.hpp
  g2o/core/parameter.cpp
  g2o/core/parameter.h
  g2o/core/cache.cpp
  g2o/core/cache.h
  g2o/core/optimizable_graph.cpp
  g2o/core/optimizable_graph.h
  g2o/core/solver.cpp
  g2o/core/solver.h
  g2o/core/creators.h
  g2o/core/optimization_algorithm_factory.cpp
  g2o/core/estimate_propagator.cpp
  g2o/core/optimization_algorithm_factory.h
  g2o/core/estimate_propagator.h
  g2o/core/factory.cpp
  g2o/core/optimization_algorithm_property.h
  g2o/core/factory.h
  g2o/core/sparse_block_matrix.h
  g2o/core/sparse_optimizer.cpp
  g2o/core/sparse_block_matrix.hpp
  g2o/core/sparse_optimizer.h
  g2o/core/hyper_dijkstra.cpp
  g2o/core/hyper_dijkstra.h
  g2o/core/parameter_container.cpp
  g2o/core/parameter_container.h
  g2o/core/optimization_algorithm.cpp
  g2o/core/optimization_algorithm.h
  g2o/core/optimization_algorithm_with_hessian.cpp
  g2o/core/optimization_algorithm_with_hessian.h
  g2o/core/optimization_algorithm_levenberg.cpp
  g2o/core/optimization_algorithm_levenberg.h
  g2o/core/optimization_algorithm_gauss_newton.cpp
  g2o/core/optimization_algorithm_gauss_newton.h
  g2o/core/jacobian_workspace.cpp
  g2o/core/jacobian_workspace.h
  g2o/core/robust_kernel.cpp
  g2o/core/robust_kernel.h
  g2o/core/robust_kernel_factory.cpp
  g2o/core/robust_kernel_factory.h
  g2o/core/robust_kernel_impl.cpp
  g2o/core/robust_kernel_impl.h
  #stuff
  g2o/stuff/string_tools.h
  g2o/stuff/color_macros.h
  g2o/stuff/macros.h
  g2o/stuff/timeutil.cpp
  g2o/stuff/misc.h
  g2o/stuff/timeutil.h
  g2o/stuff/os_specific.c
  g2o/stuff/os_specific.h
  g2o/stuff/string_tools.cpp
  g2o/stuff/property.cpp
  g2o/stuff/property.h
)
