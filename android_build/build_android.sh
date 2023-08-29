#!/bin/bash

RED="\033[1;31m"
GREEN="\033[32m"
DEFAULT_COLOR="\e[0m"

ANDROID_NDK_ROOT="/home/${USER}/Android/Sdk/ndk/25.1.8937393"
SLAM_ROOT=$(pwd)
ANDROID_ABI="arm64-v8a" 
ANDROID_PLATFORM="30"
EIGEN_VERSION="3.4.0"
MAKE="make -j4"
SKIP_DEPS=0

# Enable errexit option
set -e -o pipefail

# Define a function to handle errors
handle_error() {
    echo "An error occurred. Exiting..."
    exit 1
}

# Set up the trap to call the handle_error function
trap 'handle_error' ERR

# help menu
function script_help {
    echo "Usage: ./android_build/build_android.sh [parameters]"
    echo -e "  --app_path\t\t\tAbsolute path to DemoApp root"
    echo -e "  --ndk_path\t\t<optional> Absolute path to NDK (default is /home/${USER}/Android/Sdk/ndk/25.1.8937393)"
    echo -e "  --abi\t\t<optional> android abi to build for (arm64-v8a, armeabi-v7a, x86, x86_64). default is arm64-v8a"
    echo -e "  --platform\t\t<optional> android api level to build for (default is 30)"
    echo -e "  --clean\t\t<optional> Clean build files before building"
    echo -e "  --skip_deps\t\t<optional> Skip building and configuration of dependencies."
    echo -e "  --help\t\t\tsee this help"
}

# reset build artifcats
function clean_build {
  cd $SLAM_ROOT/Thirdparty
  rm -r -f eigen-$EIGEN_VERSION/build g2o/build DBoW2/build $SLAM_ROOT/build
  cd $SLAM_ROOT
}

# parse options from user
while [[ $# -gt 0 ]];
do
    opt="$1";
    shift;
    case "$opt" in
        "--app_path") APP_PATH="$1"; shift;;
        "--ndk_path") ANDROID_NDK_ROOT="$1"; shift;;
        "--abi") ANDROID_ABI="$1"; shift;;
        "--platform") ANDROID_PLATFORM="$1"; shift;;
        "--clean") clean_build;;
        "--skip_deps") SKIP_DEPS=1;;
        "--help") script_help; exit 0;;
        *) echo -e "${RED}Unknown parameter ${opt} {$DEFAULT_COLOR}"; exit 1;;
    esac
done

# Check if required parameters are provided
if [ -z "$APP_PATH" ]; then
    echo -e "${RED}Error: The --app_path parameter is required.${DEFAULT_COLOR}"
    script_help
    exit 0
fi

case "$ANDROID_ABI" in
    "arm64-v8a" ) OPENSSL_ANDROID_ABI="android-arm64";;
    "armeabi-v7a") OPENSSL_ANDROID_ABI="android-arm";;
    "x86") OPENSSL_ANDROID_ABI="android-x86";;
    "x86_64") OPENSSL_ANDROID_ABI="android-x86_64";;
    *) echo -e "${RED}Bad Android ABI ${opt}, use --help for more info{$DEFAULT_COLOR}"; exit 1;;
esac


CMAKE_COMMAND="cmake -B build -S . \
	-DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK_ROOT/build/cmake/android.toolchain.cmake \
	-DANDROID_ABI="$ANDROID_ABI" \
	-DANDROID_PLATFORM=android-$ANDROID_PLATFORM\
	-DANDROID_NDK=$ANDROID_NDK_ROOT\
	-DEIGEN3_INCLUDE_DIR=$SLAM_ROOT/Thirdparty/eigen-$EIGEN_VERSION
	-DBoost_INCLUDE_DIRS=$SLAM_ROOT/Thirdparty/Boost-for-Android/build/out/$ANDROID_ABI/include/boost-1_82\
	-DOpenCV_INCLUDE_DIRS=$SLAM_ROOT/Thirdparty/OpenCV-android-sdk/sdk/native/jni/include\
	-DOpenCV_LIBS=$SLAM_ROOT/Thirdparty/OpenCV-android-sdk/sdk/native/libs/$ANDROID_ABI/libopencv_java4.so\
	-DOpenssl_INCLUDE_DIR=$SLAM_ROOT/Thirdparty/openssl/include
"

if ! command -v pv &> /dev/null; then
    echo "Installing pv..."
    sudo apt-get update
    sudo apt-get install -y pv
else
    echo "pv is already installed."
fi

if [ $SKIP_DEPS == 0 ]; then

cd $SLAM_ROOT/Thirdparty

echo "Building Thirdparty libs"

#Boost
echo "Building Boost"
cd Boost-for-Android
chmod +x ./build-android.sh
./build-android.sh --boost=1.82.0 --progress --arch=$ANDROID_ABI --target-version=$ANDROID_PLATFORM $ANDROID_NDK_ROOT || handle_error
cd ..

#Eigen
echo "Building Eigen3"
cd eigen-$EIGEN_VERSION
$CMAKE_COMMAND
cd build
$MAKE
cd ../..

#openSSL
echo "Building openSSl"
cd openssl
export ANDROID_NDK_HOME=$ANDROID_NDK_ROOT
PATH=$ANDROID_NDK_ROOT/toolchains/llvm/prebuilt/linux-x86_64/bin:$ANDROID_NDK_ROOT/toolchains/arm-linux-androideabi-4.9/prebuilt/linux-x86_64/bin:$PATH
./Configure $OPENSSL_ANDROID_ABI -D__ANDROID_API__=$ANDROID_PLATFORM --prefix=$(pwd)
$MAKE
cd ..

# g2o
echo "Building g2o"
cd g2o
$CMAKE_COMMAND
cd build
$MAKE

# DBoW2
echo "Building DBoW2"
cd ../../DBoW2
$CMAKE_COMMAND
cd build
$MAKE
fi # $SKIP_DEPS == 0

# Build ORB_SLAM3 lib
echo "Building ORB_SLAM3"
cd $SLAM_ROOT
$CMAKE_COMMAND
cd build
$MAKE
cd $SLAM_ROOT

echo "Copying Shared Objects"
JNI_PATH=$APP_PATH/ServiceApp/com.LibbaInc.ltd/app/src/main/jniLibs/$ANDROID_ABI
mkdir -p $JNI_PATH
cp -r -f -v -a Thirdparty/g2o/lib/libg2o.so $JNI_PATH
cp -r -f -v -a Thirdparty/openssl/libcrypto.so $JNI_PATH
cp -r -f -v -a Thirdparty/openssl/libcrypto.so.1.1 $JNI_PATH
cp -r -f -v -a lib/* $JNI_PATH

echo "Copying Headers"
APP_HEADERS_PATH=$APP_PATH/ServiceApp/com.LibbaInc.ltd/app/src/main/jni
SLAM_HEADERS_PATH=$APP_HEADERS_PATH/SLAM
mkdir -p $SLAM_HEADERS_PATH
cp -r -f -v include/* $SLAM_HEADERS_PATH
DBOW2_HEADERS_PATH=$SLAM_HEADERS_PATH/Thirdparty/DBoW2
mkdir -p $DBOW2_HEADERS_PATH
cp -r -f -v Thirdparty/DBoW2/DBoW2 Thirdparty/DBoW2/DUtils  $DBOW2_HEADERS_PATH
G2O_HEADERS_PATH=$SLAM_HEADERS_PATH/Thirdparty/g2o
mkdir -p $G2O_HEADERS_PATH
cp -r -f -v Thirdparty/g2o/g2o Thirdparty/g2o/config.h $G2O_HEADERS_PATH
SOPHUS_HEADERS_PATH=$SLAM_HEADERS_PATH/Thirdparty/Sophus
mkdir -p $SOPHUS_HEADERS_PATH
cp -r -f -v Thirdparty/Sophus/sophus $SOPHUS_HEADERS_PATH
EIGEN_HEADERS_PATH=$APP_HEADERS_PATH
mkdir -p $EIGEN_HEADERS_PATH
cp -r -f -v Thirdparty/eigen-$EIGEN_VERSION/Eigen $EIGEN_HEADERS_PATH
BOOST_HEADERS_PATH=$APP_HEADERS_PATH
mkdir -p $BOOST_HEADERS_PATH
cp -r -f -v Thirdparty/Boost-for-Android/boost_1_82_0/boost $BOOST_HEADERS_PATH

echo "Extracting and copying ORB_SLAM3 assets"
APP_ASSETS_PATH=$APP_PATH/ServiceApp/com.LibbaInc.ltd/app/src/main/assets
tar -xzf "Vocabulary/ORBvoc.txt.tar.gz" --directory $SLAM_ROOT/android_build/assets/ORB3_SLAM || { echo "Error: Failed to extract ORBvoc.txt"; }
cp -r -f -v android_build/assets/ORB3_SLAM $APP_ASSETS_PATH
rm -f $SLAM_ROOT/android_build/assets/ORB3_SLAM/ORBvoc.txt

echo "Copying OpenCV"
OLD_VER_STR="VERSION_1_6"
NEW_VER_STR="VERSION_1_8"
OPENCV_GRADLE_PATH="${SLAM_ROOT}/Thirdparty/OpenCV-android-sdk/sdk/build.gradle"
# Search and replace in file:
sed -i "s/${OLD_VER_STR}/${NEW_VER_STR}/g" "$OPENCV_GRADLE_PATH"
cp -r -f -v Thirdparty/OpenCV-android-sdk $APP_PATH/ServiceApp

echo -e "${GREEN}SUCCESS${DEFAULT_COLOR}"
