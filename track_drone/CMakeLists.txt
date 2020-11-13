cmake_minimum_required(VERSION 2.8.4)
project(track_drone)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(sample_dir ${CMAKE_CURRENT_SOURCE_DIR}/)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ltbb -D__STDC_CONSTANT_MACROS")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")

find_package(OpenCV 3.2.0 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)

set(sample_pnplib
        ${sample_dir}motion_vectors.cpp
        ${sample_dir}Utils.cpp
        ${sample_dir}Product.cpp
        ${sample_dir}PnPProblem.cpp
        ${sample_dir}RobustMatcher.cpp
        ${sample_dir}frame.cpp
        ${sample_dir}Utils.cpp
        ${sample_dir}Projection.cpp
        ${sample_dir}PnP/PnpInput.cpp
        ${sample_dir}Utils_pnp/Parsing.cpp
        ${sample_dir}PnP/PnpObjective.cpp
        ${sample_dir}PnP/PnpProblemSolver.cpp
        ${sample_dir}PnP/DualVar.cpp
        ${sample_dir}PnP/PnpBarrierMethodSolver.cpp
        ${sample_dir}Utils_pnp/GeneralUtils.cpp
        ${sample_dir}PnP/PnpResult.cpp
        ${sample_dir}PnP/PnpLocalSearch.cpp
        ${sample_dir}PnP/QuaternionVector.cpp
        )
ocv_include_modules_recurse(${OPENCV_CPP_SAMPLES_REQUIRED_DEPS})

add_executable(track_drone main.cpp ${sample_pnplib})
include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)
include_directories(Utils_pnp PnP)

include_directories(
        -Idependencies/include
        -Ldependencies/lib
)

target_link_libraries(
        ${PROJECT_NAME}
        ${OpenCV_LIBS}
        stdc++fs -ltbb -lstdc++ -lpthread -lm -ldl -lswscale
        -lavformat -lavcodec -lswresample -lavutil -lpthread
        -lz -lc -lrt
)

target_link_libraries(track_drone ${OpenCV_LIBS} stdc++fs -ltbb -lstdc++ -lpthread -lm -ldl -lswscale -lavformat -lavcodec -lswresample -lavutil -lpthread -lz -lc -lrt)
