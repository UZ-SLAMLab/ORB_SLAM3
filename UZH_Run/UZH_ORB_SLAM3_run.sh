#!/bin/bash
excutablePathUZH='./ORB_SLAM3/UZH_Run' #Example, it is necessary to change it by the dataset path/
pathDatasetUZH='./Datasets/UZH' #Example, it is necessary to change it by the dataset path/
vocabularyPath='./ORB_SLAM3/Vocabulary/ORBvoc.txt'

#------------------------------------
# Monocular Examples
echo "Launching indoor_forward_3 with Monocular SNAP"
"$excutablePathUZH"/mono_UZH_trial "$vocabularyPath" "$excutablePathUZH"/UZH_SNAP_mono.yaml "$pathDatasetUZH"/indoor_forward_3_snapdragon "$pathDatasetUZH"/indoor_forward_3_snapdragon/left_images.txt "$pathDatasetUZH"/indoor_forward_3_snapdragon_results/mono

#------------------------------------
# Stereo Examples

#------------------------------------
# Monocular-Inertial Examples
echo "Launching indoor_forward_3 with Monocular-Inertial sensor SNAP"
"$excutablePathUZH"/mono_inertial_UZH_trial "$vocabularyPath" "$excutablePathUZH"/UZH_SNAP_mono_inertial.yaml "$pathDatasetUZH"/indoor_forward_3_snapdragon "$pathDatasetUZH"/indoor_forward_3_snapdragon/left_images.txt "$pathDatasetUZH"/indoor_forward_3_snapdragon_results/mono_inertial

#------------------------------------
# Stereo-Inertial Examples
