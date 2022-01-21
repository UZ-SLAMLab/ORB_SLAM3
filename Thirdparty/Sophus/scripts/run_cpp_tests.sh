#!/bin/bash

set -x # echo on
set -e # exit on error

mkdir build
cd build
pwd
cmake -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_BUILD_TYPE=$BUILD_TYPE ..
make -j2
make CTEST_OUTPUT_ON_FAILURE=1 test
