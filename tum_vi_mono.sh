#!/bin/bash
pathProject=$(pwd)

#------------------------------------
# Monocular Examples
echo "Launching Corridor 1 with Monocular sensor"
./Examples/Monocular/mono_tum_vi "$pathProject"/Vocabulary/ORBvoc.txt "$pathProject"/Examples/Monocular/TUM-VI.yaml "$pathProject"/Datasets/TUM_VI/dataset-corridor1_512_16/mav0/cam0/data "$pathProject"/Examples/Monocular/TUM_TimeStamps/dataset-corridor1_512.txt dataset-corridor1_512_mono