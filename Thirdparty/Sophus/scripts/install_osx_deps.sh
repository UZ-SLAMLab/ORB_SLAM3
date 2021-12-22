#!/bin/bash

set -x # echo on
set -e # exit on error
brew update
brew install eigen
brew install glog
brew install suite-sparse
brew install ccache
export PATH="/usr/local/opt/ccache/libexec:$PATH"
whereis ccache
git clone https://ceres-solver.googlesource.com/ceres-solver ceres-solver
cd ceres-solver
git reset --hard afe93546b67cee0ad205fe8044325646ed5deea9
mkdir build
cd build
ccache -M 50G
ccache -s
cmake -DCXX11=On -DCMAKE_CXX_COMPILER_LAUNCHER=ccache ..
make -j3
make install