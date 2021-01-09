cmake_minimum_required(VERSION 3.18)

project(DBoW2 VERSION 2.0.0)

set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

find_package(OpenCV CONFIG REQUIRED)

add_library(${PROJECT_NAME} STATIC)

add_library(ORB_SLAM::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_sources(${PROJECT_NAME} 
               PRIVATE 
               ${SRCS_DBOW2} 
               ${SRCS_DUTILS})

target_include_directories(${PROJECT_NAME} 
                           PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../>
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/DBoW2>
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/DUtils>
                           $<INSTALL_INTERFACE:include/>
                           $<INSTALL_INTERFACE:include/ThirdParty/DBoW2/DBoW2>
                           $<INSTALL_INTERFACE:include/ThirdParty/DBoW2/DUtils>
                           ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

install(DIRECTORY DBoW2
        DESTINATION include/ThirdParty/DBoW2/
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY DUtils
        DESTINATION include/ThirdParty/DBoW2/
        FILES_MATCHING PATTERN "*.h")
