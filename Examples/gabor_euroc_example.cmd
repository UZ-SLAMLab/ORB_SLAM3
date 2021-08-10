@echo off

set EXAMPLES_DIR=%~dp0
call %EXAMPLES_DIR%\windows-run-config.cmd

:: Example, change it to your dataset path:
set pathDatasetEuroc="C:\data\EuRoC"

:: NOTE: you must have the OpenCV DLLs reachable on your PATH, or you need to copy them into the folder of the executables

::------------------------------------
:: Monocular Examples
echo "Launching MH01 with Monocular sensor"

%ORBSLAM3_BIN_DIR%\mono_euroc.exe ^
  %ORB_VOCABULARY% ^
  %EXAMPLES_DIR%\Monocular\EuRoC.yaml ^
  %pathDatasetEuroc%\MH01 ^
  %EXAMPLES_DIR%\Monocular\EuRoC_TimeStamps\MH01.txt ^
  dataset-MH01_mono



::------------------------------------
:: Monocular-Inertial Examples
echo "Launching MH01 with Monocular-Inertial sensor"

%ORBSLAM3_BIN_DIR%\mono_inertial_euroc.exe ^
  %ORB_VOCABULARY% ^
  %EXAMPLES_DIR%\Monocular-Inertial\EuRoC.yaml ^
  %pathDatasetEuroc%\MH01 ^
  %EXAMPLES_DIR%\Monocular-Inertial\EuRoC_TimeStamps\MH01.txt ^
  dataset-MH01_monoi
