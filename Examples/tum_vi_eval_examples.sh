#!/bin/bash

# Check if all three arguments are provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <datasetPath>"
    echo "example: $0 $HOME/Datasets/TUM_VI"
    exit 1
fi

pathDatasetTUM_VI="$1" 
echo "Dataset Path: $pathDatasetTUM_VI"

# Single Session Example

echo "Launching Magistrale 1 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt ./Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-magistrale1_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-magistrale1_512_16/mav0/cam1/data ./Stereo-Inertial/TUM_TimeStamps/dataset-magistrale1_512.txt ./Stereo-Inertial/TUM_IMU/dataset-magistrale1_512.txt dataset-magistrale1_512_stereoi
echo "------------------------------------"
echo "Evaluation of Magistrale 1 trajectory with Stereo-Inertial sensor"
python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM_VI"/magistrale1_512_16/mav0/mocap0/data.csv f_dataset-magistrale1_512_stereoi.txt --plot magistrale1_512_stereoi.pdf
