#!/bin/bash

set -x # echo on
set -e # exit on error

cmake --version

sudo apt-get -qq update
sudo apt-get install gfortran libc++-dev libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.4/eigen-3.3.4.tar.bz2
tar xvf eigen-3.3.4.tar.bz2
mkdir build-eigen
cd build-eigen
cmake ../eigen-3.3.4 -DEIGEN_DEFAULT_TO_ROW_MAJOR=$ROW_MAJOR_DEFAULT
sudo make install
git clone https://ceres-solver.googlesource.com/ceres-solver ceres-solver
cd ceres-solver
git reset --hard afe93546b67cee0ad205fe8044325646ed5deea9
mkdir build
cd build
ccache -M 50G
ccache -s
cmake -DCXX11=On -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DOPENMP=Off ..
make -j3
sudo make install
