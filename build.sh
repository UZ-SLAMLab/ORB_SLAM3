#!/bin/bash
# Usage: ./build.sh <parallel_jobs> <build_type> <ros>
#        parallel_jobs: -jx where x is the number of threads
#        build_type   : could be one of the following: Release or Debug
#        ros          : could be ROS or left blank for standard compilation

BUILD_TYPE=$2

if [ "$1" == "" ]; then
  echo "No argument set for parallel jobs! Set -jx where x is the number of threads!"
  exit
elif [ "$BUILD_TYPE" == "" ]; then
  echo "No argument set for build type! Set Release or Debug. Now compiling in Release mode"
  BUILD_TYPE="Release"
fi

echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE
make $1

# echo "Configuring and building Thirdparty/fbow ..."
# cd Thirdparty/fbow
# mkdir -p build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_INSTALL_PREFIX=../ -DUSE_AVX=OFF
# make install $1

# echo "Configuring and building Thirdparty/DLib ..."
# cd ../../DLib
# mkdir -p build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE
# make $1

echo "Configuring and building Thirdparty/g2o ..."
cd ../../g2o
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE
make $1

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
VOCABULARYFILE=`pwd`"/ORBvoc.txt"
if test -f "$VOCABULARYFILE"; then
  echo "Vocabulary file already extracted."
else
  echo "Uncompress vocabulary ..."
  tar -xf ORBvoc.txt.tar.gz
fi
cd ..

# cd Vocabulary
# VOCABULARYFILE=`pwd`"/orb_mur.fbow"
# if test -f "$VOCABULARYFILE"; then
#   echo "Vocabulary file already extracted."
# else
#   echo "Uncompress vocabulary ..."
#   tar -xf orb_mur.fbow.tar.gz
# fi
# cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir -p build
cd build
if [ "$BUILD_TYPE" == "Release" ] || [ "$BUILD_TYPE" == "Debug" ]; then
  cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE
else
  echo "[ERROR] Invalid build type. Should be one of the following: Release/Debug."
  exit 1
fi
make $1

cd ..

if [ "$3" == "ROS" ]; then
  echo "Building ROS nodes"

  cd Examples/ROS/ORB_SLAM3
  mkdir -p build
  cd build
  cmake .. -DROS_BUILD_TYPE=$BUILD_TYPE
  make $1
fi
