#!/bin/bash
pathDatasetEuroc='/' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Monocular Examples
echo "Launching MH01 with Monocular sensor"
./Monocular/mono_euroc ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml "$pathDatasetEuroc"/MH04 ./Monocular/EuRoC_TimeStamps/MH04.txt dataset-MH04_mono