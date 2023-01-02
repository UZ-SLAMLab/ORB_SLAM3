cmake_minimum_required(VERSION 2.8)
project(DBoW2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# clang doesn't like
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# for M1
# see https://stackoverflow.com/questions/52879026/cmake-cross-compile-on-macos-adds-macos-sdk-to-isysroot-in-flags-make
# and https://stackoverflow.com/questions/65511230/how-do-i-use-cmake-on-os-x-target-the-new-arm-m1
#SET(CMAKE_C_FLAGS "-target arm64-apple-macos10.5")



# MAC BOOST: brew info boost
# on M1, it's in /opt/homebrew/Cellar
# todo do this properly
#set(BOOST_INCLUDE /opt/homebrew/Cellar/boost/1.80.0/include)
#set(BOOST_LIBS /opt/homebrew/Cellar/boost/1.80.0/lib)
set(BOOST_BASE /opt/homebrew/opt/boost)
# on intel, it's /usr/local/opt/boost)
set(BOOST_INCLUDE ${BOOST_BASE}/include)
set(BOOST_LIBS ${BOOST_BASE}/lib)
#set(BOOST_INCLUDE /usr/local/Cellar/boost/1.80.0/include)
#set(BOOST_LIBS /usr/local/Cellar/boost/1.80.0/lib)

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

