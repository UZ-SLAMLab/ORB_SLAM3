CMAKE_MINIMUM_REQUIRED(VERSION 3.18)
# SET(CMAKE_LEGACY_CYGWIN_WIN32 0)

PROJECT(g2o VERSION 0.0.1)

# Find Eigen3
find_package(Eigen3 CONFIG REQUIRED)

# Include the subdirectories
add_library(${PROJECT_NAME} STATIC)

add_library(ORB_SLAM::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

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