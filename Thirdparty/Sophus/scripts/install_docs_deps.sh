#!/bin/bash

set -x # echo on
set -e # exit on error

sudo apt-get -qq update
sudo apt-get install doxygen liblua5.3-dev
pip3 install 'sphinx==2.0.1'
pip3 install sphinx_rtd_theme
pip3 install sympy

git clone https://github.com/vovkos/doxyrest_b
cd doxyrest_b
git reset --hard ad45c064d1199e71b8cae5aa66d4251c4228b958
git submodule update --init
mkdir build
cd build
cmake ..
cmake --build .

cd ../..