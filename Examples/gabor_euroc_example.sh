#!/bin/bash

# Example, change it to your dataset path:
pathDatasetEuroc='/home/esoptron/data/EuRoC'

#------------------------------------
# Monocular Examples
echo "Launching MH01 with Monocular sensor"
./Monocular/mono_euroc \
    ../Vocabulary/ORBvoc.txt \
    ./Monocular/EuRoC.yaml \
    "$pathDatasetEuroc"/MH01 \
    ./Monocular/EuRoC_TimeStamps/MH01.txt \
    dataset-MH01_mono

#------------------------------------
# Monocular-Inertial Examples
echo "Launching MH01 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_euroc \
    ../Vocabulary/ORBvoc.txt \
    ./Monocular-Inertial/EuRoC.yaml \
    "$pathDatasetEuroc"/MH01 \
    ./Monocular-Inertial/EuRoC_TimeStamps/MH01.txt \
    dataset-MH01_monoi

