#!/bin/bash

# Check if all three arguments are provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <datasetPath>"
    echo "example: $0 $HOME/Datasets/EuRoC"
    exit 1
fi

pathDatasetEuroc="$1" 
#------------------------------------
# Monocular Examples
echo "Launching MH01 with Monocular sensor"
./Monocular/mono_euroc ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml "$pathDatasetEuroc"/MH_01_easy ./Monocular/EuRoC_TimeStamps/MH01.txt dataset-MH01_mono

echo "Launching MH02 with Monocular sensor"
./Monocular/mono_euroc ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml "$pathDatasetEuroc"/MH_02_easy ./Monocular/EuRoC_TimeStamps/MH02.txt dataset-MH02_mono

echo "Launching MH03 with Monocular sensor"
./Monocular/mono_euroc ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml "$pathDatasetEuroc"/MH_03_medium ./Monocular/EuRoC_TimeStamps/MH03.txt dataset-MH03_mono

echo "Launching MH04 with Monocular sensor"
./Monocular/mono_euroc ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml "$pathDatasetEuroc"/MH_04_difficult ./Monocular/EuRoC_TimeStamps/MH04.txt dataset-MH04_mono

echo "Launching MH05 with Monocular sensor"
./Monocular/mono_euroc ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml "$pathDatasetEuroc"/MH_05_difficult ./Monocular/EuRoC_TimeStamps/MH05.txt dataset-MH05_mono

echo "Launching V101 with Monocular sensor"
./Monocular/mono_euroc ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml "$pathDatasetEuroc"/V1_01_easy ./Monocular/EuRoC_TimeStamps/V101.txt dataset-V101_mono

echo "Launching V102 with Monocular sensor"
./Monocular/mono_euroc ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml "$pathDatasetEuroc"/V1_02_medium ./Monocular/EuRoC_TimeStamps/V102.txt dataset-V102_mono

echo "Launching V103 with Monocular sensor"
./Monocular/mono_euroc ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml "$pathDatasetEuroc"/V1_03_difficult ./Monocular/EuRoC_TimeStamps/V103.txt dataset-V103_mono

echo "Launching V201 with Monocular sensor"
./Monocular/mono_euroc ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml "$pathDatasetEuroc"/V2_01_easy ./Monocular/EuRoC_TimeStamps/V201.txt dataset-V201_mono

echo "Launching V202 with Monocular sensor"
./Monocular/mono_euroc ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml "$pathDatasetEuroc"/V2_02_medium ./Monocular/EuRoC_TimeStamps/V202.txt dataset-V202_mono

echo "Launching V203 with Monocular sensor"
./Monocular/mono_euroc ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml "$pathDatasetEuroc"/V2_03_difficult ./Monocular/EuRoC_TimeStamps/V203.txt dataset-V203_mono

# MultiSession Monocular Examples
echo "Launching Machine Hall with Monocular sensor"
./Monocular/mono_euroc ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml "$pathDatasetEuroc"/MH01 ./Monocular/EuRoC_TimeStamps/MH01.txt "$pathDatasetEuroc"/MH02 ./Monocular/EuRoC_TimeStamps/MH02.txt "$pathDatasetEuroc"/MH03 ./Monocular/EuRoC_TimeStamps/MH03.txt "$pathDatasetEuroc"/MH04 ./Monocular/EuRoC_TimeStamps/MH04.txt "$pathDatasetEuroc"/MH05 ./Monocular/EuRoC_TimeStamps/MH05.txt dataset-MH01_to_MH05_mono

echo "Launching Vicon Room 1 with Monocular sensor"
./Monocular/mono_euroc ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml "$pathDatasetEuroc"/V101 ./Monocular/EuRoC_TimeStamps/V101.txt "$pathDatasetEuroc"/V102 ./Monocular/EuRoC_TimeStamps/V102.txt "$pathDatasetEuroc"/V103 ./Monocular/EuRoC_TimeStamps/V103.txt dataset-V101_to_V103_mono

echo "Launching Vicon Room 2 with Monocular sensor"
./Monocular/mono_euroc ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml "$pathDatasetEuroc"/V201 ./Monocular/EuRoC_TimeStamps/V201.txt "$pathDatasetEuroc"/V202 ./Monocular/EuRoC_TimeStamps/V202.txt "$pathDatasetEuroc"/V203 ./Monocular/EuRoC_TimeStamps/V203.txt dataset-V201_to_V203_mono
