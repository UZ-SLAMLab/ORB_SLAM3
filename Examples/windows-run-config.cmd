@echo off

set EXAMPLES_DIR=%~dp0
set ORBSLAM3_ROOT_DIR=%EXAMPLES_DIR%\..

:: Update these paths according to your directory structure
set ORBSLAM3_DEPENDENCIES_INSTALL_DIR="C:\dev\collabslam\install"
set ORBSLAM3_BIN_DIR=%ORBSLAM3_ROOT_DIR%\bin
set ORB_VOCABULARY=%ORBSLAM3_ROOT_DIR%\Vocabulary\ORBvoc.txt

set PATH=%ORBSLAM3_BIN_DIR%;%ORBSLAM3_DEPENDENCIES_INSTALL_DIR%\bin;%ORBSLAM3_DEPENDENCIES_INSTALL_DIR%\x64\vc16\bin;%PATH%
