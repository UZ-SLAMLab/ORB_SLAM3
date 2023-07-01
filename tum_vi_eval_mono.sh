#!/bin/bash
pathProject=$(pwd)

echo "Evaluation"
python evaluation/evaluate_ate_scale.py "$pathProject"/Datasets/TUM_VI/dataset-corridor1_512_16/mav0/mocap0/data.csv "$pathProject"/f_dataset-corridor1_512_mono.txt --plot dataset-corridor1_512_mono.pdf