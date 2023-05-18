import json
import argparse
import subprocess
import os
from pathlib import Path
from typing import List, Optional, Dict
import shutil
import csv
import logging
import zipfile

ROOT_DIR = "/home/codebind/robotics/orbslam3"

REPO_PATH = f'{ROOT_DIR}/ORB_SLAM3/'
PATH_TO_DATASETS = f"{ROOT_DIR}/datasets/"
METRIC_INDEX = 2

ALLOWED_SCORES = [
    "L1", # L1Scoring
    "L2", # L2Scoring
    "CHI", # ChiSquareScoring
    "KL", # KLScoring
    "BA", # BhattacharyyaScoring
    "DOT", # DotProductScoring,
    "JS", # Jensen–Shannon divergence Scoring
]

ALLOWED_ROOMS = [
    "MH03",
    "MH05"
]

EVO_CMD = "evo_ape"
FILE_FORMAT = "tum"

MODIFIED_GT_PATH = f"{ROOT_DIR}/benchmark/GT"


def validate_paths(paths: List[Path]) -> None:
    for p in paths:
        assert os.path.exists(p), f"path {p} does not exist"

def create_if_not_exists(path: Path) -> None:
    os.makedirs(path, exist_ok=True)

def validate_score(s: str) -> None:
    assert s in ALLOWED_SCORES


def validate_room(r: str) -> None:
    assert r in ALLOWED_ROOMS

def print_command(args: List) -> None:
    printable_command = " ".join(args)
    logging.info(f"Running {printable_command}")

def get_new_voc_path(voc_path: Path, output_path: Path, score: str):
    """
    Get current vocabulary path, copy and modify it according to score type
    """
    assert os.path.exists(voc_path), f"path {voc_path} does not exist"
    suff = Path(voc_path).name
    
    with open(voc_path, 'r') as f:
        lines = f.readlines()
    
    first_line: str = lines[0]
    args: List = first_line.split()
    args[METRIC_INDEX] = str(ALLOWED_SCORES.index(score))
    lines[0] = " ".join(args) + "\n"
    
    # save the modified vocabulary text in new_voc_path
    new_voc_path = os.path.join(output_path, suff)
    with open(new_voc_path, 'w') as fp:
        for l in lines:
            fp.write(l)
    
    return new_voc_path    

def parse_log_file_for_rmse(plot_path: Path):
    validate_paths([plot_path])
    with open(plot_path, 'r') as f:
        lines = f.readlines()
    
    rmse_lines = []
    for l in lines:
        if 'rmse' in l:
            rmse_lines.append(l)
            
    assert len(rmse_lines) == 1
    rmse_line = (rmse_lines[0]).split()
    assert len(rmse_line) == 2
    assert rmse_line[0] == 'rmse'
    return rmse_line[1]

def parse_results_file(results_path: Path):
    validate_paths([results_path])
    z = zipfile.ZipFile(results_path, 'r')
    zipped_files = z.namelist()
    assert zipped_files[1] == 'stats.json'
    bitim = z.read(zipped_files[1])
    results = json.loads(bitim)
    
    return results['rmse'], results['mean'], results['std'], results['min'], results['max'], results['sse']

def execute_room_and_metric(
    room: str,
    score: str,
    output_path: Path,
    align: Optional[bool],
    scale: Optional[bool],
    benchmark_file: Optional[Path],
):
    
    logging.basicConfig(format='%(asctime)s - %(message)s', level=logging.INFO)
    
    logging.info("Running...")
    logging.info(f"Will evaluate: room: {room}, score: {score}")
    
    # prepare ORBSlam3 args
    validate_room(room)
    validate_score(score)
    create_if_not_exists(output_path)
    
    examples_path = os.path.join(REPO_PATH, 'Examples', "Monocular")
    
    # script path : ./Monocular/mono_euroc
    script_path = os.path.join(examples_path, "mono_euroc")
    
    # voc path: ../Vocabulary/ORBvoc.txt
    voc_path = os.path.join(REPO_PATH, "Vocabulary", "ORBvoc.txt")
    new_voc_path = get_new_voc_path(voc_path, output_path, score)
    
    # configuration path : ./Monocular/EuRoC.yaml
    config_path = os.path.join(examples_path, "EuRoC.yaml")
    
    # path to dataset: "$pathDatasetEuroc"/MH01
    dataset_path = os.path.join(PATH_TO_DATASETS, room)
    
    # path to timestamps: ./Monocular/EuRoC_TimeStamps/MH01.txt
    tm_stamps_path = os.path.join(examples_path, "EuRoC_TimeStamps", f"{room}.txt")
    
    # give a name <name> that will be f_<name>.txt under current dir
    f_name = f"room_{room}_score_{score}"
    
    paths = [
        script_path,
        new_voc_path,
        config_path,
        dataset_path,
        tm_stamps_path,
    ]
    validate_paths(paths)
    args = paths
    args.append(f_name)
    
    # Run OBSlam3
    print_command(args)
    subprocess.call(args=args)
    
    # prepare evo args
    
    # ground truth path, modified
    gt_path = os.path.join(MODIFIED_GT_PATH, f"{room}_GT.txt")
    
    # estimated path, moved from current directory to the output path
    est_path = f"f_{f_name}.txt"
    validate_paths([gt_path, est_path])
    shutil.move(est_path, output_path)
    new_est_path = os.path.join(output_path, est_path)
    validate_paths([new_est_path])
    
    args = [
        EVO_CMD,
        FILE_FORMAT,
        gt_path,
        new_est_path,
    ]
    
    if align:
        args.append("-a")
    if scale:
        args.append("-s")
    
    plot_path = os.path.join(output_path, "plot")
    result_path = os.path.join(output_path, "result.zip")
    args.extend([
        "--plot",
        "--plot_mode",
        "xyz",
        "--save_plot",
        plot_path,
        "--logfile",
        plot_path,
        "--save_results",
        result_path
    ])
    
    print_command(args)
    subprocess.call(args=args)
    
    if benchmark_file:
        rows_to_add = []
        if not os.path.exists(benchmark_file):
            rows_to_add.append(["score", "room", "rmse", "mean", "median", "std", "min", "max", "sse"])
        
        parsed_results = parse_results_file(result_path)
        row = [score, room, *parsed_results]
        rows_to_add.append(row)
        logging.info(f"Writing results to {benchmark_file}")
        with open(benchmark_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerows(rows_to_add)


if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--room', help="room to run ORBSlam3 on", type=str, required=True)
    parser.add_argument('--score', help="score method for Bow comparison", type=str, required=True)
    parser.add_argument('--align', help="align estimated coord system to ground truth", action="store_true")
    parser.add_argument('--scale', help="scale estimated coord system", action="store_true")
    parser.add_argument('--output_path', help="path to dump outputs", required=True)
    parser.add_argument("--benchmark_file", help="path to csv, will add the rmse to the csv", required=False)
    args = parser.parse_args()

    execute_room_and_metric(
        room=args.room,
        score=args.score,
        output_path=args.output_path,
        align=args.align,
        scale=args.scale,
        benchmark_file=args.benchmark_file,
    )
