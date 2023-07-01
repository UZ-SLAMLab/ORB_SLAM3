#!/bin/bash
pathProject=$(pwd)

# Stereo Examples
echo "Launching Room 1 with Stereo sensor"
./Examples/Stereo/stereo_tum_vi "$pathProject"/Vocabulary/ORBvoc.txt Examples/Stereo/TUM-VI.yaml "$pathProject"/Datasets/TUM_VI/dataset-corridor1_512_16/mav0/cam0/data "$pathProject"/Datasets/TUM_VI/dataset-corridor1_512_16/mav0/cam1/data "$pathProject"/Examples/Stereo/TUM_TimeStamps/dataset-corridor1_512.txt dataset-corridor1_512_stereo
