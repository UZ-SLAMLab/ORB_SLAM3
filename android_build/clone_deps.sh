#!/bin/bash

RED="\033[1;31m"
GREEN="\033[32m"
DEFAULT_COLOR="\e[0m"

BUILD_DIR=$(pwd)
EIGEN_VERSION="3.4.0"

# Enable errexit option
set -e -o pipefail

# reset Thirdparty
function delete_libs {
  cd $BUILD_DIR/../Thirdparty
  rm -r -f eigen-$EIGEN_VERSION Boost-for-Android OpenCV-android-sdk openssl
}

# Define a function to handle errors
handle_error() {
    echo "An error occurred. Exiting..."
    read -p "Delete downloaded libs? (Y/N): " DELETE
    if [ "$DELETE" == "Y" ]; then
      delete_libs
    fi
    exit 1
}

# Set up the trap to call the handle_error function
trap 'handle_error' ERR

# help menu
function script_help {
    echo "Usage: clone_deps.sh [parameters]"
    echo -e "  -f<optional>\t\t\tOverwrite existing files and folders"
    echo -e "  --help\t\t\tsee this help"
}


# parse options from user
while [[ $# -gt 0 ]];
do
    opt="$1";
    shift;
    case "$opt" in
        "-f") delete_libs;;
        "--help") script_help; exit 0;;
        *) echo -e "${RED}Unknown parameter ${opt} {$DEFAULT_COLOR}"; exit 1;;
    esac
done

cd ../Thirdparty

echo "Downloading & Building Thirdparty libs"

#Eigen
if [ -d "eigen-$EIGEN_VERSION" ]; then
  echo "Eigen $EIGEN_VERSION already exists in the current directory."
else
# Download Eigen tar.gz file
echo "Downloading Eigen $EIGEN_VERSION..."
wget -P . https://gitlab.com/libeigen/eigen/-/archive/$EIGEN_VERSION/eigen-$EIGEN_VERSION.tar.gz
# Extract the contents of the tar.gz file to the current directory
echo "Extracting Eigen $EIGEN_VERSION..."
tar -xzf "eigen-$EIGEN_VERSION.tar.gz" || { echo "Error: Failed to extract Eigen."; }
rm eigen-$EIGEN_VERSION.tar.gz
fi

#Boost
if [ -d "Boost-for-Android" ]; then
  echo "Boost-for-Android already exists in the current directory."
else
echo "Cloning Boost-for-Android"
git clone git@github.com:moritz-wundke/Boost-for-Android.git || { echo "Error: Failed to clone Boost-for-Android."; }
cd Boost-for-Android
git checkout c6012c576e30ff6000ddab0988d59bad849200ce 
cd ..
fi

#OpenCV
if [ -d "OpenCV-android-sdk" ]; then
  echo "OpenCV-android-sdk already exists in the current directory."
else
echo "Downloading OpenCV-android-sdk..."
wget -q -O opencv-4.4.0-android-sdk.zip -P . https://downloads.sourceforge.net/project/opencvlibrary/4.4.0/opencv-4.4.0-android-sdk.zip?ts=gAAAAABkykXlBXKEi_aBv5z2QAjG9b7whPfUXwv-xPehw0nHGiCK7tBHy1gtQnjGhsWu3vN_SwUS2wsAeI6KYKTsVOocHwModQ%3D%3D&r=https%3A%2F%2Fsourceforge.net%2Fprojects%2Fopencvlibrary%2Ffiles%2F4.8.0%2Fopencv-4.8.0-android-sdk.zip%2Fdownload || { echo "Error: Failed to download OpenCV."; }
sleep 30 # wget has a bug that somtimes makes the download a background process.
# Extract the contents of the zip file to the current directory
echo "Extracting OpenCV..."
unzip opencv-4.4.0-android-sdk.zip || { sleep 10; unzip opencv-4.4.0-android-sdk.zip; }|| { sleep 10; unzip opencv-4.4.0-android-sdk.zip; }|| { sleep 10; unzip opencv-4.4.0-android-sdk.zip; }|| { echo "Error: Failed to extract OpenCV."; }
rm opencv-4.4.0-android-sdk.zip
fi

#openSSL
if [ -d "openssl" ]; then
  echo "openssl already exists in the current directory."
else
echo "Cloning openssl..."
git clone git@github.com:openssl/openssl.git || { echo "Error: Failed to clone openssl."; }
cd openssl
git checkout 70c2912 # version 1.1.1u
cd ..
fi