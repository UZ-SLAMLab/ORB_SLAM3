#!/bin/bash
currentDir="$(pwd)"
runType=Monocular
datasetType=MH_01_easy
timestampType=MH01

pathDatasetEuroc="data/EuRoC" #Example, it is necesary to change it by the dataset path

#------------------------------------
# Monocular Examples
echo "Launching $timestampType with $runType sensor"

./Examples/$runType/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/$runType/EuRoC.yaml $pathDatasetEuroc/$datasetType ./Examples/$runType/EuRoC_TimeStamps/$timestampType.txt "$datasetType_$runType"