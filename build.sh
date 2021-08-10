# Set this path. Dependencies are assumed to be installed to this folder:
#ORBSLAM3_DEPENDENCIES_INSTALL_DIR="/home/esoptron/dev/slamtest/install"
ORBSLAM3_DEPENDENCIES_INSTALL_DIR=""
echo "ORBSLAM3_DEPENDENCIES_INSTALL_DIR=$ORBSLAM3_DEPENDENCIES_INSTALL_DIR"


echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=$ORBSLAM3_DEPENDENCIES_INSTALL_DIR

make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=$ORBSLAM3_DEPENDENCIES_INSTALL_DIR

make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."

mkdir build
cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=$ORBSLAM3_DEPENDENCIES_INSTALL_DIR

make -j

cd ..
