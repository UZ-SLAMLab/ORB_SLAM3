#!/bin/bash
pathDatasetEuroc='/home/thomas/EuRoC' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Stereo Save Example
echo "Launching MH01 with Stereo sensor for Map Saving"
./Stereo/stereo_save ../Vocabulary/ORBvoc.txt ./Stereo/EuRoC.yaml /home/thomas/EuRoC/MH01 ./Stereo/EuRoC_TimeStamps/MH01.txt MH01_save

#------------------------------------
# Stereo Load Example
echo "Launching MH01 with Stereo sensor for Map Loading"
./Stereo/stereo_load ../Vocabulary/ORBvoc.txt ./Stereo/EuRoC.yaml /home/thomas/EuRoC/MH01 ./Stereo/EuRoC_TimeStamps/MH01.txt MH01_save

