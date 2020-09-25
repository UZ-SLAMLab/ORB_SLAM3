cmake_minimum_required(VERSION 2.8)
project(DBoW2)

# Compiler specific options for gcc
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")
endif()
# Compiler specific options for MSVC
if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /W3")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3")
endif()

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

set(DBOW2_LIB_TYPE SHARED)
IF(WIN32)
  SET(DBOW2_LIB_TYPE STATIC)
ENDIF(WIN32)

add_library(DBoW2 ${DBOW2_LIB_TYPE} ${SRCS_DBOW2} ${SRCS_DUTILS})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# add OpenCV
find_package(OpenCV 3 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 4 QUIET)
  if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
  endif()
endif()
MESSAGE(STATUS "OpenCV VERSION: ${OpenCV_VERSION}")
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

# add Boost
find_package(Boost REQUIRED COMPONENTS serialization)
MESSAGE(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")
target_link_libraries(DBoW2 ${Boost_LIBRARIES})
