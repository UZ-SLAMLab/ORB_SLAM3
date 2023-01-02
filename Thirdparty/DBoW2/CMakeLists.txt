cmake_minimum_required(VERSION 2.8)
project(DBoW2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# for M1
# see https://stackoverflow.com/questions/52879026/cmake-cross-compile-on-macos-adds-macos-sdk-to-isysroot-in-flags-make
# and https://stackoverflow.com/questions/65511230/how-do-i-use-cmake-on-os-x-target-the-new-arm-m1
# need to run with -DCMAKE_APPLE_SILICON_PROCESSOR=arm64
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")

# set up clang-specific stuff

if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
  MESSAGE("Using Clang")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
else()
        MESSAGE("Not using CLANG!")
        # clang doesn't like this setting
        set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
endif()

# M1 specific stuff
# homebrew puts things in different places, depending:
# on M1, it's in /opt/homebrew/
# on intel, it's /usr/local/opt/boost
# this also impacts openssl
#see https://stackoverflow.com/questions/35477940/linking-statically-openssl-crypto-library-in-cmake


IF ("${CMAKE_HOST_SYSTEM_PROCESSOR}" MATCHES "arm64")
        MESSAGE("HOST: " ${CMAKE_HOST_SYSTEM_PROCESSOR})
        set (BOOST_BASE /opt/homebrew)
        # openssl is put in a strange place by homebrew
        set (OPENSSL_ROOT_DIR /opt/homebrew/opt/openssl@3)
ELSE()
        MESSAGE("NOT ARM64")
        set(BOOST_BASE /usr/local/opt/boost)
        set (OPENSSL_ROOT_DIR /usr/local/opt/openssl@3)
ENDIF()

# these are used in the find_package Boost command later
set(BOOST_INCLUDE ${BOOST_BASE}/include)        #should be BOOST_INCLUDEDIR?
set(BOOST_LIBS ${BOOST_BASE}/lib)

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


find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 3.0 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 3.0 not found.")
   endif()
endif()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS} ${BOOST_INCLUDE})
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS} ${BOOST_LIBS})

