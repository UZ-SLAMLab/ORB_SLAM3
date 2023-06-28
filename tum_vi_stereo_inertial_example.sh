#!/bin/bash
pathDatasetTUM_VI='/Datasets/TUM_VI' #Example, it is necesary to change it by the dataset path


#------------------------------------
# Stereo-Inertial Examples
echo "Launching Corridor 1 with Stereo-Inertial sensor"
./Examples/Stereo-Inertial/stereo_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-corridor1_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-corridor1_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-corridor1_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-corridor1_512.txt dataset-corridor1_512_stereoi

echo "Launching Corridor 2 with Stereo-Inertial sensor"
