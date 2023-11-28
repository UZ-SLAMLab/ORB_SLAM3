#!/bin/bash

# Check if all three arguments are provided
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <Environment> <Track> <Camera>"
    echo "example: $0 SeaFloor track0 cam0"
    exit 1
fi

echo "Environment: $1"
echo "Track: $2"
echo "Camera: $3"

data_path="$HOME/Datasets/MIMIR/$1/$2/auv0/rgb/$3/data"
timestamps_path="Examples/Monocular/MIMIR_TimeStamps/$1_$2.txt"

echo $data_path
echo $timestamps_path

# Add quotes around $1_$2_$3.txt to handle cases where the arguments contain spaces
./Examples/Monocular/mono_mimir Vocabulary/ORBvoc.txt Examples/Monocular/MIMIR.yaml "$data_path" "$timestamps_path" "$1_$2_$3.txt"
