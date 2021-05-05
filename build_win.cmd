@echo off

:: NOTE: dependencies are assumed to be installed to this folder:
set ORBSLAM3_DEPENDENCIES_INSTALL_DIR=""

set CMAKE_EXECUTABLE="C:\Program Files\CMake\bin\cmake.exe"
set GENERATOR_NAME="Visual Studio 16 2019" 
set GENERATOR_PLATFORM="x64"


@echo "Uncompress vocabulary ..."
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..


@echo Configuring and building ORB_SLAM3 ...

mkdir build
cd build

%CMAKE_EXECUTABLE% .. ^
  -G%GENERATOR_NAME% ^
  -A%GENERATOR_PLATFORM% ^
  -DCMAKE_PREFIX_PATH=%ORBSLAM3_DEPENDENCIES_INSTALL_DIR%

%CMAKE_EXECUTABLE% --build . --config Release
@echo Building Release done.

cd..
