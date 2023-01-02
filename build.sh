set -e
echo "Configuring and building Thirdparty/DBoW2 ..."

# note that uname -m returns intel when running on a M1 mac if it's running under Rosetta
# this is very annoying
EXTRA_CMAKE_ARGS="-DCMAKE_APPLE_SILICON_PROCESSOR=`uname -m`"

echo Running with extra cmake args: ${EXTRA_CMAKE_ARGS}

cd Thirdparty/DBoW2
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release ${EXTRA_CMAKE_ARGS}
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_TYPE=Release ${EXTRA_CMAKE_ARGS}
make -j

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_TYPE=Release ${EXTRA_CMAKE_ARGS}
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_TYPE=Release ${EXTRA_CMAKE_ARGS}
make -j4
