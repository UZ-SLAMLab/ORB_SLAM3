# Make sure Ceres knows where to find Eigen
list(APPEND SEARCH_HEADERS ${EIGEN3_INCLUDE_DIR})

# git clone https://ceres-solver.googlesource.com/ceres-solver
find_package( Ceres 1.6.0 QUIET )

if( Ceres_FOUND )
  MESSAGE(STATUS "CERES found")

  # Tests to run
  SET( TEST_SOURCES test_ceres_se3 )

  FOREACH(test_src ${TEST_SOURCES})
    ADD_EXECUTABLE( ${test_src} ${test_src}.cpp local_parameterization_se3)
    TARGET_LINK_LIBRARIES( ${test_src} sophus ${CERES_LIBRARIES} )
    TARGET_INCLUDE_DIRECTORIES( ${test_src} SYSTEM PRIVATE ${CERES_INCLUDE_DIRS})
    ADD_TEST( ${test_src} ${test_src} )
  ENDFOREACH(test_src)

endif( Ceres_FOUND )
