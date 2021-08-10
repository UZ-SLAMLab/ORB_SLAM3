cmake_minimum_required(VERSION 2.8)
set(DBOW2_TARGET "DBoW2")
project(${DBOW2_TARGET})

set(DBOW2_LIB_TYPE SHARED)
if(WIN32)
  set(DBOW2_LIB_TYPE STATIC)
endif(WIN32)


# Compiler options
# Compiler specific options for gcc
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")
endif()
# Compiler specific options for MSVC
if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /W3")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3")
endif()


# Target definition
set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h
  DBoW2/FClass.h
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

add_library(${DBOW2_TARGET} ${DBOW2_LIB_TYPE} ${SRCS_DBOW2} ${SRCS_DUTILS})


# Dependencies

# add OpenCV
find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3 QUIET)
  if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
  endif()
endif()
MESSAGE(STATUS "OpenCV VERSION: ${OpenCV_VERSION}")
#include_directories(${OpenCV_INCLUDE_DIRS}) # not needed in modern cmake
target_link_libraries(${DBOW2_TARGET} PUBLIC ${OpenCV_LIBS})

# add Boost
find_package(Boost REQUIRED COMPONENTS serialization)
message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")
target_link_libraries(${DBOW2_TARGET} PUBLIC ${Boost_LIBRARIES})
