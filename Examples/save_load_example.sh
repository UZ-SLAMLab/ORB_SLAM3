#!/bin/bash
pathDatasetEuroc='/home/thomas/EuRoC' #Example, it is necesary to change it by the dataset path

# Monocular Save Example
#------------------------------------
echo "Launching MH01 with Monocular sensor for Map Saving"
./Monocular/mono_save ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml ${pathDatasetEuroc}/MH01 ./Monocular/EuRoC_TimeStamps/MH01.txt MH01_mono_save

# Monocular Load Example
#------------------------------------
echo "Launching MH01 with Monocular sensor for Map Loading"
./Monocular/mono_load ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml ${pathDatasetEuroc}/MH01 ./Monocular/EuRoC_TimeStamps/MH01.txt MH01_mono_save

'
#------------------------------------
# Stereo Save Example
echo "Launching MH01 with Stereo sensor for Map Saving"
./Stereo/stereo_save ../Vocabulary/ORBvoc.txt ./Stereo/EuRoC.yaml ${pathDatasetEuroc}/MH01 ./Stereo/EuRoC_TimeStamps/MH01.txt MH01_stereo_save

#------------------------------------
# Stereo Load Example
echo "Launching MH01 with Stereo sensor for Map Loading"
./Stereo/stereo_load ../Vocabulary/ORBvoc.txt ./Stereo/EuRoC.yaml ${pathDatasetEuroc}/MH01 ./Stereo/EuRoC_TimeStamps/MH01.txt MH01_stereo_save
