#!/bin/bash
# Set paths
pathVocabulary='/home/justmohsen/Documents/SLAM/ORBSLAM/Vocabulary/ORBvoc.txt'
pathDatasetPixel='/home/justmohsen/Documents/SLAM/Datasets/Pixel6/HomeFlying/Concorde_Place-2023-11-30_03-42-23'
cameraTimestamp='/home/justmohsen/Documents/SLAM/Datasets/Pixel6/HomeFlying/Concorde_Place-2023-11-30_03-42-23/Camera/Camera0.txt'
pathPixelRun='/home/justmohsen/Documents/SLAM/ORBSLAM/Run_Examples/pixel6pro'
pathMonocularResults='/home/justmohsen/Documents/SLAM/Datasets/Pixel6/HomeFlying/Concorde_Place-2023-11-30_03-42-23/ORBSLAM_Results/monocular'
pathMonocularInertialResults='/home/justmohsen/Documents/SLAM/Datasets/Pixel6/HomeFlying/Concorde_Place-2023-11-30_03-42-23/ORBSLAM_Results/monocular-inertial'

#------------------------------------
# Monocular Examples
echo "Launching Monocular SLAM"
"$pathPixelRun"/mono_pixel "$pathVocabulary" "$pathPixelRun"/Pixel_mono.yaml "$pathDatasetPixel" "$cameraTimestamp" "$pathMonocularResults"
#------------------------------------
# Monocular-Inertial Examples
echo "Launching Monocular-Inertial SLAM"
"$pathPixelRun"/mono_inertial_pixel "$pathVocabulary" "$pathPixelRun"/Pixel_Mono_Inertial.yaml "$pathDatasetPixel" "$cameraTimestamp" "$pathMonocularInertialResults"
