#!/home/lab405/henry/ORB_SLAM3/slam_env/bin/python3
"""
ORB-SLAM3 Unified Benchmark Script
====================================
支援以下演算法：
  mono            – mono_euroc / mono_tum_vi
  mono_inertial   – mono_inertial_euroc / mono_inertial_tum_vi
  stereo          – stereo_euroc / stereo_tum_vi
  stereo_inertial – stereo_inertial_euroc / stereo_inertial_tum_vi

可執行的步驟（flags）：
  --run        執行 SLAM binary，產生原始軌跡 f_<name>.txt
  --fix-time   將時間戳 ns → s，產生 fix_<name>.txt
  --eval       以 evo_ape 計算 APE RMSE，結果存入 benchmark_results_<ts>.csv
  --plot-traj  繪製軌跡比對圖（evaluate_ate_scale.py）
  --plot-3d    繪製 XY / 3D / XZ / YZ 四視圖 PDF
  --plot-diag  繪製 per-frame ATE + tracking 指標診斷圖

  不加任何 step flag → 全部步驟依序執行（預設行為）

使用方式：
  cd /home/lab405/henry/for_git/ORB_SLAM3/evaluation

  # 全跑（原本行為）
  python3 run_stereo_inertial_benchmark.py

  # 只跑 SLAM binary（不做後處理）
  python3 run_stereo_inertial_benchmark.py --run

  # 只跑 binary + fix-time
  python3 run_stereo_inertial_benchmark.py --run --fix-time

  # 已有 fix_*.txt，只重跑評估和所有圖
  python3 run_stereo_inertial_benchmark.py --eval --plot-traj --plot-3d --plot-diag

  # 只重繪診斷圖
  python3 run_stereo_inertial_benchmark.py --plot-diag

  # 篩選演算法 / 資料集（可與 step flags 組合）
  python3 run_stereo_inertial_benchmark.py --algo stereo_inertial --dataset euroc
  python3 run_stereo_inertial_benchmark.py --algo stereo --dataset euroc --plot-3d --plot-diag
"""

import argparse
import copy
import csv
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.patches as mpatches
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    import numpy as np
    import pandas as pd
    from evo.tools import file_interface
    from evo.core import sync as evo_sync
    import copy
    _PLOT_AVAILABLE = True
except ImportError as _e:
    _PLOT_AVAILABLE = False
    print(f"[WARN] 繪圖套件不可用，跳過軌跡圖與診斷圖：{_e}")

# ─────────────────────────────────────────────
#  基礎路徑
# ─────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).parent.resolve()
ROOT_DIR   = SCRIPT_DIR.parent

VOCAB       = ROOT_DIR / "Vocabulary/ORBvoc.txt"
EXAMPLES    = ROOT_DIR / "Examples"
FIX_TIME_PY = SCRIPT_DIR / "fix_time.py"

TUM_DATA         = ROOT_DIR / "/media/lab405/Windows1/data/TUM"
EUROC_DATA       = ROOT_DIR / "/media/lab405/Windows1/data/Euroc"
EVALUATE_ATE_PY  = SCRIPT_DIR / "evaluate_ate_scale.py"

GT_EUROC_LEFT_CAM = SCRIPT_DIR / "Ground_truth/EuRoC_left_cam"
GT_EUROC_IMU      = SCRIPT_DIR / "Ground_truth/EuRoC_imu"

# ─────────────────────────────────────────────
#  演算法設定
#
#  tum_extra(mav0_path) → 插入在 times_file 之前的額外引數列表
#    mono              : []
#    mono_inertial     : [imu_file]
#    stereo            : [cam1]
#    stereo_inertial   : [cam1, imu_file]
# ─────────────────────────────────────────────

# 插在 times_file 之前的額外引數（僅 cam1）
def _tum_pre_mono(mav0: Path) -> list:
    return []

def _tum_pre_mono_inertial(mav0: Path) -> list:
    return []

def _tum_pre_stereo(mav0: Path) -> list:
    return [str(mav0 / "cam1/data")]

def _tum_pre_stereo_inertial(mav0: Path) -> list:
    return [str(mav0 / "cam1/data")]

# 插在 times_file 之後的額外引數（imu CSV）
def _tum_post_mono(mav0: Path) -> list:
    return []

def _tum_post_mono_inertial(mav0: Path) -> list:
    return [str(mav0 / "imu0/data.csv")]

def _tum_post_stereo(mav0: Path) -> list:
    return []

def _tum_post_stereo_inertial(mav0: Path) -> list:
    return [str(mav0 / "imu0/data.csv")]


ALGO_CONFIGS: dict = {
    "mono": {
        "label":        "Mono",
        "bin_dir":      EXAMPLES / "Monocular",
        "bin_euroc":    "mono_euroc",
        "bin_tum":      "mono_tum_vi",
        "yaml_euroc":   EXAMPLES / "Monocular/EuRoC.yaml",
        "yaml_tum":     EXAMPLES / "Monocular/TUM-VI.yaml",
        "euroc_ts_dir": EXAMPLES / "Monocular/EuRoC_TimeStamps",
        "tum_ts_dir":   EXAMPLES / "Monocular/TUM_TimeStamps",
        "tum_pre":      _tum_pre_mono,             # 插在 times 前
        "tum_post":     _tum_post_mono,            # 插在 times 後
        "out_prefix_e": "mono_euroc",
        "out_prefix_t": "mono_tum",
        "evo_align":    "-as",                     # monocular: align + scale
        "euroc_gt_type": "left_cam",               # GT 來源：left camera frame
        "out_subdir":   "mono",
    },
    "mono_inertial": {
        "label":        "Mono-Inertial",
        "bin_dir":      EXAMPLES / "Monocular-Inertial",
        "bin_euroc":    "mono_inertial_euroc",
        "bin_tum":      "mono_inertial_tum_vi",
        "yaml_euroc":   EXAMPLES / "Monocular-Inertial/EuRoC.yaml",
        "yaml_tum":     EXAMPLES / "Monocular-Inertial/TUM-VI.yaml",
        "euroc_ts_dir": EXAMPLES / "Monocular-Inertial/EuRoC_TimeStamps",
        "tum_ts_dir":   EXAMPLES / "Monocular-Inertial/TUM_TimeStamps",
        "tum_pre":      _tum_pre_mono_inertial,    # 插在 times 前（無）
        "tum_post":     _tum_post_mono_inertial,   # 插在 times 後（imu）
        "out_prefix_e": "mimu_euroc",
        "out_prefix_t": "mimu_tum",
        "evo_align":    "-a",                      # inertial: scale known, align only
        "euroc_gt_type": "imu",                    # GT 來源：IMU/body frame
        "out_subdir":   "mono_inertial",
    },
    "stereo": {
        "label":        "Stereo",
        "bin_dir":      EXAMPLES / "Stereo",
        "bin_euroc":    "stereo_euroc",
        "bin_tum":      "stereo_tum_vi",
        "yaml_euroc":   EXAMPLES / "Stereo/EuRoC.yaml",
        "yaml_tum":     EXAMPLES / "Stereo/TUM-VI.yaml",
        "euroc_ts_dir": EXAMPLES / "Stereo/EuRoC_TimeStamps",
        "tum_ts_dir":   EXAMPLES / "Stereo/TUM_TimeStamps",
        "tum_pre":      _tum_pre_stereo,           # 插在 times 前（cam1）
        "tum_post":     _tum_post_stereo,          # 插在 times 後（無）
        "out_prefix_e": "stereo_euroc",
        "out_prefix_t": "stereo_tum",
        "evo_align":    "-a",                      # stereo: align only
        "euroc_gt_type": "left_cam",               # GT 來源：left camera frame
        "out_subdir":   "stereo",
    },
    "stereo_inertial": {
        "label":        "Stereo-Inertial",
        "bin_dir":      EXAMPLES / "Stereo-Inertial",
        "bin_euroc":    "stereo_inertial_euroc",
        "bin_tum":      "stereo_inertial_tum_vi",
        "yaml_euroc":   EXAMPLES / "Stereo-Inertial/EuRoC.yaml",
        "yaml_tum":     EXAMPLES / "Stereo-Inertial/TUM-VI.yaml",
        "euroc_ts_dir": EXAMPLES / "Stereo-Inertial/EuRoC_TimeStamps",
        "tum_ts_dir":   EXAMPLES / "Stereo-Inertial/TUM_TimeStamps",
        "tum_pre":      _tum_pre_stereo_inertial,  # 插在 times 前（cam1）
        "tum_post":     _tum_post_stereo_inertial, # 插在 times 後（imu）
        "out_prefix_e": "stereo_imu_euroc",
        "out_prefix_t": "stereo_imu_tum",
        "evo_align":    "-a",                      # stereo: align only
        "euroc_gt_type": "imu",                    # GT 來源：IMU/body frame
        "out_subdir":   "stereo_imu",
    },
}

# ─────────────────────────────────────────────
#  資料集清單
#  EuRoC : (seq_folder, ts_stem, output_suffix)
#  TUM-VI: (category,   seq_name, output_suffix)
# ─────────────────────────────────────────────
EUROC_DATASETS = [
    # ("V1_01_easy",      "V101", "v101"),
    ("V1_02_medium",    "V102", "v102"),
    # ("V1_03_difficult", "V103", "v103"),
    # ("V2_01_easy", "V201", "v201"),
    # ("V2_02_medium", "V202", "v202"),
    # ("V2_03_difficult", "V203", "v203"),
    # ("MH_01_easy", "MH01", "m01"),
    # ("MH_02_easy", "MH02", "m02"),
    # ("MH_03_medium", "MH03", "m03"),
    # ("MH_04_difficult", "MH04", "m04"),
    # ("MH_05_difficult", "MH05", "m05"),
]

TUM_DATASETS = [
    # room
    # ("room",     "room1",     "room1"),
    # ("room",     "room2",     "room2"),
    # ("room",     "room3",     "room3"),
    # ("room",     "room4",     "room4"),
    # ("room",     "room5",     "room5"),
    # ("room",     "room6",     "room6"),
    # # corridor
    # ("corridor", "corridor1", "corr1"),
    # ("corridor", "corridor2", "corr2"),
    # ("corridor", "corridor3", "corr3"),
    # ("corridor", "corridor4", "corr4"),
    # ("corridor", "corridor5", "corr5"),
    # # slides
    # ("slides",   "slides1",   "slides1"),
    # ("slides",   "slides2",   "slides2"),
    # ("slides",   "slides3",   "slides3"),
    # # magistrale
    # ("magistrale", "magistrale1", "mag1"),
    # ("magistrale", "magistrale2", "mag2"),
    # ("magistrale", "magistrale3", "mag3"),
    # ("magistrale", "magistrale4", "mag4"),
    # ("magistrale", "magistrale5", "mag5"),
    #outdoors
    ("outdoors", "outdoors6", "out6"),
]

# ─────────────────────────────────────────────
#  輔助函式
# ─────────────────────────────────────────────

def log(msg: str) -> None:
    print(msg, flush=True)


def _seq_to_subdir(seq_suffix: str) -> str:
    """Convert sequence suffix (e.g. 'm01') to output subdirectory name ('MH_01')."""
    m = re.match(r'^m(\d+)$', seq_suffix)
    if m:
        return f"MH_{m.group(1).zfill(2)}"
    return seq_suffix


def _get_out_dir(cfg: dict, seq_suffix: str) -> Path:
    """Compute (and create) the per-run output subdirectory."""
    out_dir = SCRIPT_DIR / cfg["out_subdir"] / _seq_to_subdir(seq_suffix)
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir


def run_cmd(cmd: list, cwd=None, timeout: int = 7200):
    """執行外部指令，回傳 (stdout, stderr, returncode)"""
    cmd_str = " ".join(str(c) for c in cmd)
    log(f"  >> {cmd_str}")
    result = subprocess.run(
        [str(c) for c in cmd],
        cwd=str(cwd) if cwd else None,
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    return result.stdout, result.stderr, result.returncode


def parse_evo_rmse(stdout: str, stderr: str) -> float | None:
    """從 evo_ape 輸出解析 RMSE 數值"""
    combined = stdout + stderr
    print("=== evo_ape raw output ===")
    print(combined[:1000])          # ← 加這行看原始輸出
    print("=========================")
    match = re.search(r'rmse\s+([\d.eE+\-]+)', combined)
    if match:
        return float(match.group(1))
    return None


def resolve_euroc_gt(ts_stem: str, gt_type: str) -> str:
    """
    根據 ts_stem（如 'V102'、'V201'、'MH01'）和 gt_type 找到本地 GT 檔案路徑。

    gt_type='left_cam'：
        GT_EUROC_LEFT_CAM/{ts_stem}_GT.txt
        例：V102 → Ground_truth/EuRoC_left_cam/V102_GT.txt

    gt_type='imu'：
        GT_EUROC_IMU/{group}_GT.txt，group 取 ts_stem 前 2 字元
        例：V102 → V1_GT.txt；V201 → V2_GT.txt；MH01 → MH_GT.txt
    """
    if gt_type == "left_cam":
        return str(GT_EUROC_LEFT_CAM / f"{ts_stem}_GT.txt")
    else:  # imu
        group = ts_stem[:2]   # "V1", "V2", "MH"
        return str(GT_EUROC_IMU / f"{group}_GT.txt")


def resolve_euroc_seq_path(seq_folder: str) -> Path | None:
    """解析 EuRoC 序列實際路徑（處理巢狀資料夾情況）"""
    base = EUROC_DATA / seq_folder
    if (base / "mav0").exists():
        return base
    nested = base / seq_folder
    if (nested / "mav0").exists():
        return nested
    return None


def fix_time(raw_traj: Path, fixed_traj: Path) -> bool:
    """呼叫 fix_time.py 轉換時間戳 ns → s"""
    stdout, stderr, rc = run_cmd(
        [sys.executable, FIX_TIME_PY, raw_traj, fixed_traj]
    )
    if rc != 0:
        log(f"  [ERROR] fix_time.py 失敗:\n{stderr[:500]}")
        return False
    return True


def eval_with_evo(gt_csv: str, fixed_traj: Path, align_flag: str = "-a") -> float | None:
    """以 evo_ape euroc 計算 RMSE（align_flag: '-a' 或 '-as'）"""
    stdout, stderr, rc = run_cmd(
        ["evo_ape", "euroc", gt_csv, str(fixed_traj), align_flag],
        cwd=SCRIPT_DIR,
    )
    if rc != 0:
        log(f"  [ERROR] evo_ape 失敗:\n{stderr[:500]}")
        return None
    rmse = parse_evo_rmse(stdout, stderr)
    if rmse is None:
        log(f"  [WARN] 無法從 evo_ape 輸出解析 RMSE\n{stdout[:300]}")
    return rmse


def convert_gt_to_tum(gt_csv: str, out_path: Path) -> bool:
    """將 EuRoC/TUM-VI GT CSV (timestamp_ns, px, py, pz, qw, qx, qy, qz, ...)
    轉換為 evaluate_ate_scale.py 所需的 TUM 格式 (timestamp_s tx ty tz qx qy qz qw)。"""
    try:
        with open(gt_csv, "r") as fin, open(out_path, "w") as fout:
            for line in fin:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split(',')
                if len(parts) < 8:
                    continue
                ts_s = float(parts[0]) / 1e9
                px, py, pz = parts[1].strip(), parts[2].strip(), parts[3].strip()
                # EuRoC/TUM-VI GT 四元數順序為 qw, qx, qy, qz；TUM 格式需要 qx, qy, qz, qw
                qw, qx, qy, qz = parts[4].strip(), parts[5].strip(), parts[6].strip(), parts[7].strip()
                fout.write(f"{ts_s:.9f} {px} {py} {pz} {qx} {qy} {qz} {qw}\n")
        return True
    except Exception as e:
        log(f"  [WARN] GT 格式轉換失敗: {e}")
        return False


def plot_trajectory_3d_views(gt_csv: str, fixed_traj: Path, output_name: str, out_dir: Path) -> None:
    """用 evo 產生 XY / 3D / XZ / YZ 四視圖 PDF（來自 ate_3d.py）。"""
    if not _PLOT_AVAILABLE:
        return
    out_pdf = out_dir / f"traj3d_{output_name}.pdf"
    try:
        traj_ref = file_interface.read_euroc_csv_trajectory(gt_csv)
        traj_est = file_interface.read_tum_trajectory_file(str(fixed_traj))
        traj_ref, traj_est = evo_sync.associate_trajectories(traj_ref, traj_est, 0.01)
        traj_est_aligned = copy.deepcopy(traj_est)
        traj_est_aligned.align(traj_ref, correct_scale=False)

        gt_x  = traj_ref.positions_xyz[:, 0]
        gt_y  = traj_ref.positions_xyz[:, 1]
        gt_z  = traj_ref.positions_xyz[:, 2]
        px    = traj_est_aligned.positions_xyz[:, 0]
        py    = traj_est_aligned.positions_xyz[:, 1]
        pz    = traj_est_aligned.positions_xyz[:, 2]

        ts = traj_est_aligned.timestamps
        t0 = ts[0]
        n  = len(px)
        colors = plt.cm.rainbow(np.linspace(0, 1, max(n - 1, 1)))
        interval = max(1, n // 10)

        def _add_time_labels(ax_2d, xs, ys):
            for i in range(0, n, interval):
                ax_2d.annotate(f"{ts[i]-t0:.1f}s", (xs[i], ys[i]),
                               fontsize=7, color="dimgray",
                               xytext=(4, 4), textcoords="offset points")

        with PdfPages(str(out_pdf)) as pdf:
            # XY（時間漸層）
            fig, ax = plt.subplots(figsize=(10, 8))
            ax.plot(gt_x, gt_y, "k-", linewidth=2, label="Ground Truth")
            for i in range(n - 1):
                ax.plot(px[i:i+2], py[i:i+2], color=colors[i], linewidth=2.0)
            ax.plot(px[0], py[0], "go", markersize=8, label="Start")
            ax.plot(px[-1], py[-1], "rs", markersize=8, label="End")
            _add_time_labels(ax, px, py)
            sm = plt.cm.ScalarMappable(cmap="rainbow",
                                       norm=plt.Normalize(vmin=0, vmax=ts[-1]-t0))
            sm.set_array([]); plt.colorbar(sm, ax=ax, label="Elapsed time (s)", shrink=0.7)
            ax.axis("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
            ax.set_title(f"Trajectory XY — {output_name} (color=time)")
            ax.legend(); ax.grid(False)
            pdf.savefig(fig, bbox_inches="tight"); plt.close(fig)

            # 3D（時間漸層）
            fig3 = plt.figure(figsize=(10, 8))
            ax3  = fig3.add_subplot(111, projection="3d")
            ax3.plot(gt_x, gt_y, gt_z, "k-", linewidth=2, label="Ground Truth")
            for i in range(n - 1):
                ax3.plot(px[i:i+2], py[i:i+2], pz[i:i+2], color=colors[i], linewidth=2.0)
            ax3.scatter(px[0], py[0], pz[0], c="green", s=60, zorder=5, label="Start")
            ax3.scatter(px[-1], py[-1], pz[-1], c="red", s=60, marker="s", zorder=5, label="End")
            for i in range(0, n, interval):
                ax3.text(px[i], py[i], pz[i], f"{ts[i]-t0:.1f}s", fontsize=7, color="dimgray")
            ax3.set_xlabel("x [m]"); ax3.set_ylabel("y [m]"); ax3.set_zlabel("z [m]")
            ax3.set_title(f"Trajectory 3D — {output_name} (color=time)"); ax3.legend()
            try: ax3.set_box_aspect((1, 1, 1))
            except AttributeError: pass
            pdf.savefig(fig3, bbox_inches="tight"); plt.close(fig3)

            # XZ
            fig_xz, ax_xz = plt.subplots(figsize=(10, 8))
            ax_xz.plot(gt_x, gt_z, "k-", linewidth=2, label="Ground Truth")
            for i in range(n - 1):
                ax_xz.plot(px[i:i+2], pz[i:i+2], color=colors[i], linewidth=2.0)
            _add_time_labels(ax_xz, px, pz)
            ax_xz.axis("equal"); ax_xz.set_xlabel("x [m]"); ax_xz.set_ylabel("z [m]")
            ax_xz.set_title(f"Trajectory XZ — {output_name} (color=time)")
            ax_xz.legend(); ax_xz.grid(False)
            pdf.savefig(fig_xz, bbox_inches="tight"); plt.close(fig_xz)

            # YZ
            fig_yz, ax_yz = plt.subplots(figsize=(10, 8))
            ax_yz.plot(gt_y, gt_z, "k-", linewidth=2, label="Ground Truth")
            for i in range(n - 1):
                ax_yz.plot(py[i:i+2], pz[i:i+2], color=colors[i], linewidth=2.0)
            _add_time_labels(ax_yz, py, pz)
            ax_yz.axis("equal"); ax_yz.set_xlabel("y [m]"); ax_yz.set_ylabel("z [m]")
            ax_yz.set_title(f"Trajectory YZ — {output_name} (color=time)")
            ax_yz.legend(); ax_yz.grid(False)
            pdf.savefig(fig_yz, bbox_inches="tight"); plt.close(fig_yz)

        log(f"  [PLOT] 3D 軌跡圖已儲存至: {out_pdf.name}")
    except Exception as e:
        log(f"  [WARN] 3D 軌跡圖失敗: {e}")


def _load_gt_positions(gt_csv: str) -> pd.DataFrame:
    rows = []
    with open(gt_csv) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 4:
                continue
            try:
                ts = float(parts[0]) / 1e9
                rows.append({"timestamp": ts,
                             "px": float(parts[1]),
                             "py": float(parts[2]),
                             "pz": float(parts[3])})
            except ValueError:
                continue
    return pd.DataFrame(rows)


def _load_traj_positions(traj_path: Path) -> pd.DataFrame:
    rows = []
    with open(traj_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            rows.append({"timestamp": float(parts[0]),
                         "px": float(parts[1]),
                         "py": float(parts[2]),
                         "pz": float(parts[3])})
    return pd.DataFrame(rows)


def _align_se3(est_pos: np.ndarray, gt_pos: np.ndarray):
    mu_e = est_pos.mean(0); mu_g = gt_pos.mean(0)
    H = (est_pos - mu_e).T @ (gt_pos - mu_g)
    U, _, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    return R, mu_g - R @ mu_e


def plot_diagnosis(gt_csv: str, fixed_traj: Path, output_name: str, out_dir: Path) -> None:
    """產生 per-frame ATE + tracking 指標診斷圖（來自 analyze_trajectory.py）。"""
    if not _PLOT_AVAILABLE:
        return
    log_path = out_dir / f"{output_name}_tracking_log.csv"
    out_pdf  = out_dir / f"diagnosis_{output_name}.pdf"
    if not log_path.exists():
        log(f"  [WARN] 找不到 tracking log: {log_path.name}，跳過診斷圖")
        return
    try:
        est    = _load_traj_positions(fixed_traj)
        gt     = _load_gt_positions(gt_csv)
        log_df = pd.read_csv(log_path)

        # 對齊時間戳
        est_ts  = est["timestamp"].values
        gt_ts   = gt["timestamp"].values
        gt_pos  = gt[["px", "py", "pz"]].values
        est_pos_list, gt_pos_list, ts_list = [], [], []
        for i, ts in enumerate(est_ts):
            idx = int(np.argmin(np.abs(gt_ts - ts)))
            if np.abs(gt_ts[idx] - ts) < 0.05:
                est_pos_list.append(est[["px", "py", "pz"]].values[i])
                gt_pos_list.append(gt_pos[idx])
                ts_list.append(ts)
        if not ts_list:
            log("  [WARN] 診斷圖：無對齊幀，跳過")
            return

        ts_arr   = np.array(ts_list)
        est_pos  = np.array(est_pos_list)
        gt_pos_a = np.array(gt_pos_list)
        R, t     = _align_se3(est_pos, gt_pos_a)
        errors   = np.linalg.norm((R @ est_pos.T).T + t - gt_pos_a, axis=1)
        common_t0 = ts_arr[0]          # 軌跡第一幀的絕對時間戳為共同參考點
        rel_ts   = ts_arr - common_t0
        rmse     = float(np.sqrt(np.mean(errors ** 2)))

        log_rel  = log_df["timestamp"].values - common_t0
        state_col_name = "當前幀狀態" if "當前幀狀態" in log_df.columns else \
                         ("当前帧状态" if "当前帧状态" in log_df.columns else None)
        if state_col_name:
            lost_mask = log_df[state_col_name].isin(["LOST", "RECENTLY_LOST"]).values
        else:
            lost_mask = np.zeros(len(log_df), dtype=bool)
        reloc_mask = log_df["reloc_attempted"].astype(str).str.lower().isin(["yes", "true", "1"]).values
        method_map = {"none": 0, "RefKF": 1, "MM": 2, "IMU_MM": 3, "Reloc": 4}
        method_vals = log_df["tracking_method"].map(lambda x: method_map.get(str(x), -1)).values

        fig, axes = plt.subplots(4, 1, figsize=(16, 14), sharex=False)
        fig.suptitle(f"Trajectory Diagnosis: {output_name}", fontsize=13, fontweight="bold")

        # 子圖 1：ATE
        ax1 = axes[0]
        ax1.plot(rel_ts, errors, color="royalblue", linewidth=0.8, label="Position Error (m)")
        ax1.axhline(rmse, color="red", linestyle="--", linewidth=0.8, label=f"RMSE={rmse:.4f}m")
        ax1.set_ylabel("ATE (m)"); ax1.set_title("Per-frame Position Error (after SE3 alignment)")
        ax1.legend(loc="upper right", fontsize=8); ax1.grid(True, alpha=0.3)

        # 子圖 2：final_inliers vs matches_before_tlm_opt
        ax2 = axes[1]
        ax2.plot(log_rel, log_df["matches_before_tlm_opt"].values,
                 color="orange", linewidth=0.8, linestyle="--", label="before_opt")
        ax2.plot(log_rel, log_df["final_inliers"].values,
                 color="darkgreen", linewidth=0.8, label="final_inliers")
        ax2.set_ylabel("Inliers")
        ax2.set_title("Inliers: before opt (orange--) vs after opt (green)  |  large gap = unstable pose opt")
        ax2.legend(loc="upper right", fontsize=8); ax2.grid(True, alpha=0.3)

        # LOST 區塊
        in_lost = False; lost_start = None
        for t_i, is_lost in zip(log_rel, lost_mask):
            if is_lost and not in_lost:
                lost_start = t_i; in_lost = True
            elif not is_lost and in_lost:
                for ax in [ax1, ax2]:
                    ax.axvspan(lost_start, t_i, alpha=0.15, color="gray")
                in_lost = False
        if in_lost:
            for ax in [ax1, ax2]:
                ax.axvspan(lost_start, log_rel[-1], alpha=0.15, color="gray")

        for t_i in log_rel[reloc_mask]:
            ax2.axvline(t_i, color="red", linewidth=0.5, alpha=0.6)

        # 子圖 3：tracking_method step 折線 + 狀態背景
        ax3 = axes[2]
        ax3.step(log_rel, method_vals, where="mid", color="royalblue", linewidth=0.8)
        ax3.set_yticks(list(method_map.values()))
        ax3.set_yticklabels(list(method_map.keys()), fontsize=8)
        ax3.set_ylim(-0.5, max(method_map.values()) + 0.5)
        if state_col_name:
            rl_mask = log_df[state_col_name].astype(str) == "RECENTLY_LOST"
            lo_mask = log_df[state_col_name].astype(str) == "LOST"
            in_rl = False; in_lo = False; rl_s = lo_s = None
            for t_i, is_rl, is_lo in zip(log_rel, rl_mask.values, lo_mask.values):
                if is_rl and not in_rl:
                    rl_s = t_i; in_rl = True
                elif not is_rl and in_rl:
                    ax3.axvspan(rl_s, t_i, alpha=0.35, color="orange"); in_rl = False
                if is_lo and not in_lo:
                    lo_s = t_i; in_lo = True
                elif not is_lo and in_lo:
                    ax3.axvspan(lo_s, t_i, alpha=0.5, color="red"); in_lo = False
            if in_rl and rl_s is not None:
                ax3.axvspan(rl_s, log_rel[-1], alpha=0.35, color="orange")
            if in_lo and lo_s is not None:
                ax3.axvspan(lo_s, log_rel[-1], alpha=0.5,  color="red")
        ax3.set_ylabel("Tracking Method")
        ax3.set_title("Tracking Method (step)  |  orange=RECENTLY_LOST  red=LOST")
        ax3.legend(handles=[
            mpatches.Patch(color="orange", alpha=0.5, label="RECENTLY_LOST"),
            mpatches.Patch(color="red",    alpha=0.5, label="LOST"),
        ], loc="upper right", fontsize=7)
        ax3.grid(True, alpha=0.3)

        # 子圖 4：bias
        ax4 = axes[3]
        ax4.plot(log_rel, log_df["bias_acc_norm"].values,  color="tomato",    linewidth=0.8, label="bias_acc_norm")
        ax4.plot(log_rel, log_df["bias_gyro_norm"].values, color="steelblue", linewidth=0.8, label="bias_gyro_norm")
        ax4.set_ylabel("Bias norm"); ax4.set_xlabel("Relative time (s)")
        ax4.set_title("IMU Bias Norm (always 0 in pure visual mode)")
        ax4.legend(loc="upper right", fontsize=8); ax4.grid(True, alpha=0.3)

        t_max = max(float(log_rel[-1]), float(rel_ts[-1]))
        for ax in axes:
            ax.set_xlim(0, t_max)

        lost_patch  = mpatches.Patch(color="gray", alpha=0.3, label="LOST / RECENTLY_LOST")
        reloc_patch = mpatches.Patch(color="red",  alpha=0.6, label="Reloc attempted")
        fig.legend(handles=[lost_patch, reloc_patch], loc="lower center", ncol=2, fontsize=8)

        plt.tight_layout(rect=[0, 0.03, 1, 0.97])
        fig.savefig(str(out_pdf), dpi=150, bbox_inches="tight")
        plt.close(fig)
        log(f"  [PLOT] 診斷圖已儲存至: {out_pdf.name}  (RMSE={rmse:.6f}m, frames={len(ts_arr)})")
    except Exception as e:
        log(f"  [WARN] 診斷圖失敗: {e}")


def plot_trajectory(gt_csv: str, fixed_traj: Path, output_name: str, out_dir: Path) -> None:
    """呼叫 evaluate_ate_scale.py --plot 產生軌跡比對圖（PDF）。"""
    if not EVALUATE_ATE_PY.exists():
        log(f"  [WARN] 找不到 {EVALUATE_ATE_PY}，跳過繪圖")
        return

    gt_tum   = out_dir / f"gt_tum_{output_name}.txt"
    plot_out = out_dir / f"trajectory_{output_name}.pdf"

    if not convert_gt_to_tum(gt_csv, gt_tum):
        return

    stdout, stderr, rc = run_cmd(
        [sys.executable, str(EVALUATE_ATE_PY),
         str(gt_tum), str(fixed_traj),
         "--max_difference", "0.02",
         "--plot", str(plot_out)],
        cwd=SCRIPT_DIR,
    )
    gt_tum.unlink(missing_ok=True)   # 刪除臨時 GT 轉換檔
    if rc != 0:
        log(f"  [WARN] 軌跡繪圖失敗:\n{stderr[:300]}")
    else:
        log(f"  [PLOT] 軌跡圖已儲存至: {plot_out.name}")


def _postprocess(raw_traj: Path, output_name: str, gt_csv: str,
                 align_flag: str = "-a", steps: set = None,
                 out_dir: Path = None) -> tuple:
    """fix_time + evo_ape + 軌跡圖，回傳 (rmse_or_'FAILED', note)"""
    if steps is None:
        steps = {"fix-time", "eval", "plot-traj", "plot-3d", "plot-diag"}
    if out_dir is None:
        out_dir = SCRIPT_DIR

    fixed_traj = out_dir / f"fix_{output_name}.txt"

    if "fix-time" in steps:
        if not raw_traj.exists():
            log(f"  [ERROR] {raw_traj} 不存在")
            return "FAILED", f"找不到輸出軌跡: {raw_traj}"
        if not fix_time(raw_traj, fixed_traj):
            return "FAILED", "fix_time.py 失敗"
    else:
        if not fixed_traj.exists():
            log(f"  [SKIP] fix-time 未執行且 {fixed_traj.name} 不存在，跳過後續步驟")
            return "SKIP", "需先執行 --fix-time"

    if "plot-traj" in steps:
        plot_trajectory(gt_csv, fixed_traj, output_name, out_dir)

    if "plot-3d" in steps:
        plot_trajectory_3d_views(gt_csv, fixed_traj, output_name, out_dir)

    if "plot-diag" in steps:
        plot_diagnosis(gt_csv, fixed_traj, output_name, out_dir)

    if "eval" in steps:
        rmse = eval_with_evo(gt_csv, fixed_traj, align_flag)
        if rmse is not None:
            return rmse, "OK"
        return "FAILED", "evo_ape 無法解析 RMSE"

    return "SKIP", "未執行 eval"


# ─────────────────────────────────────────────
#  EuRoC 執行函式
#  所有 EuRoC 二進位呼叫格式相同：
#    bin  vocab  yaml  seq_path  times_file  output_name
#  （IMU 由二進位內部從 mav0/imu0 載入）
# ─────────────────────────────────────────────

def run_euroc(cfg: dict, seq_folder: str, ts_stem: str, seq_suffix: str,
              steps: set = None) -> dict:
    if steps is None:
        steps = {"run", "fix-time", "eval", "plot-traj", "plot-3d", "plot-diag"}

    output_name = f"{cfg['out_prefix_e']}_{seq_suffix}"
    out_dir     = _get_out_dir(cfg, seq_suffix)
    entry = {
        "algo":        cfg["label"],
        "dataset":     "EuRoC",
        "sequence":    seq_folder,
        "output_name": output_name,
        "rmse":        "FAILED",
        "note":        "",
    }

    seq_path = resolve_euroc_seq_path(seq_folder)
    if seq_path is None:
        entry["note"] = "序列資料夾不存在"
        log(f"  [SKIP] {EUROC_DATA / seq_folder}/mav0 不存在")
        return entry

    times_file = cfg["euroc_ts_dir"] / f"{ts_stem}.txt"
    gt_csv     = resolve_euroc_gt(ts_stem, cfg.get("euroc_gt_type", "left_cam"))
    binary     = cfg["bin_dir"] / cfg["bin_euroc"]

    if "run" in steps:
        for p, label in [(binary, "執行檔"), (times_file, "時間戳檔案"),
                         (Path(gt_csv), "Ground truth")]:
            if not Path(p).exists():
                entry["note"] = f"{label} 不存在: {p}"
                log(f"  [SKIP] {p}")
                return entry

        cmd = [binary, VOCAB, cfg["yaml_euroc"], seq_path, times_file, output_name]
        _, stderr, rc = run_cmd(cmd, cwd=ROOT_DIR)

        for log_type in ["tracking_log.csv", "localmapping_log.csv", "loop_closing_log.csv"]:
            old_log = ROOT_DIR / log_type
            if old_log.exists():
                new_log = out_dir / f"{output_name}_{log_type}"
                old_log.rename(new_log)
                log(f"  [LOG] Moved {log_type} to {new_log.relative_to(SCRIPT_DIR)}")

        if rc != 0:
            entry["note"] = f"{cfg['bin_euroc']} 回傳 {rc}"
            log(f"  [ERROR] 執行失敗:\n{stderr[-800:]}")
            return entry
    else:
        if not Path(gt_csv).exists():
            entry["note"] = f"Ground truth 不存在: {gt_csv}"
            return entry

    rmse, note = _postprocess(ROOT_DIR / f"f_{output_name}.txt", output_name, gt_csv,
                               cfg["evo_align"], steps, out_dir)
    entry["rmse"] = rmse
    entry["note"] = note
    return entry


# ─────────────────────────────────────────────
#  TUM-VI 執行函式
#  呼叫格式：bin  vocab  yaml  cam0  [extra...]  times_file  output_name
#  extra 依演算法而異（無 / imu / cam1 / cam1+imu）
# ─────────────────────────────────────────────

def run_tum(cfg: dict, category: str, seq_name: str, seq_suffix: str,
            steps: set = None) -> dict:
    if steps is None:
        steps = {"run", "fix-time", "eval", "plot-traj", "plot-3d", "plot-diag"}

    output_name = f"{cfg['out_prefix_t']}_{seq_suffix}"
    out_dir     = _get_out_dir(cfg, seq_suffix)
    entry = {
        "algo":        cfg["label"],
        "dataset":     "TUM-VI",
        "sequence":    seq_name,
        "output_name": output_name,
        "rmse":        "FAILED",
        "note":        "",
    }

    dataset_path = TUM_DATA / category / f"dataset-{seq_name}_512_16"
    mav0 = dataset_path / "mav0"
    gt_csv = str(mav0 / "mocap0/data.csv")

    if "run" in steps:
        if not mav0.exists():
            entry["note"] = f"資料集目錄不存在: {mav0}"
            log(f"  [SKIP] {mav0}")
            return entry

        cam0       = mav0 / "cam0/data"
        times_file = cfg["tum_ts_dir"] / f"dataset-{seq_name}_512.txt"
        binary     = cfg["bin_dir"] / cfg["bin_tum"]
        pre_args   = cfg["tum_pre"](mav0)
        post_args  = cfg["tum_post"](mav0)

        for p, label in [(binary, "執行檔"), (cam0, "cam0"),
                         (times_file, "timestamps"), (Path(gt_csv), "Ground truth")]:
            if not Path(p).exists():
                entry["note"] = f"{label} 不存在: {p}"
                log(f"  [SKIP] {p}")
                return entry

        for extra_p in pre_args + post_args:
            if not Path(extra_p).exists():
                entry["note"] = f"額外引數路徑不存在: {extra_p}"
                log(f"  [SKIP] {extra_p}")
                return entry

        cmd = [binary, VOCAB, cfg["yaml_tum"], cam0] + pre_args + [times_file] + post_args + [output_name]
        _, stderr, rc = run_cmd(cmd, cwd=ROOT_DIR)

        for log_type in ["tracking_log.csv", "localmapping_log.csv", "loop_closing_log.csv"]:
            old_log = ROOT_DIR / log_type
            if old_log.exists():
                new_log = out_dir / f"{output_name}_{log_type}"
                old_log.rename(new_log)
                log(f"  [LOG] Moved {log_type} to {new_log.relative_to(SCRIPT_DIR)}")

        if rc != 0:
            entry["note"] = f"{cfg['bin_tum']} 回傳 {rc}"
            log(f"  [ERROR] 執行失敗:\n{stderr[-800:]}")
            return entry
    else:
        if not Path(gt_csv).exists():
            entry["note"] = f"Ground truth 不存在: {gt_csv}"
            return entry

    rmse, note = _postprocess(ROOT_DIR / f"f_{output_name}.txt", output_name, gt_csv,
                               cfg["evo_align"], steps, out_dir)
    entry["rmse"] = rmse
    entry["note"] = note
    return entry


# ─────────────────────────────────────────────
#  主程式
# ─────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="ORB-SLAM3 Unified Benchmark")
    parser.add_argument(
        "--algo",
        choices=["mono", "mono_inertial", "stereo", "stereo_inertial", "all"],
        default="all",
        help="要測試的演算法 (預設: all)",
    )
    parser.add_argument(
        "--dataset",
        choices=["euroc", "tum", "all"],
        default="all",
        help="要測試的資料集 (預設: all)",
    )
    parser.add_argument("--run",       action="store_true", help="執行 SLAM binary")
    parser.add_argument("--fix-time",  action="store_true", help="轉換時間戳 ns→s")
    parser.add_argument("--eval",      action="store_true", help="計算 evo_ape RMSE")
    parser.add_argument("--plot-traj", action="store_true", help="繪製軌跡比對圖")
    parser.add_argument("--plot-3d",   action="store_true", help="繪製 3D 四視圖")
    parser.add_argument("--plot-diag", action="store_true", help="繪製診斷圖")
    args = parser.parse_args()

    # 若沒有指定任何 step → 全跑
    _all_steps = {"run", "fix-time", "eval", "plot-traj", "plot-3d", "plot-diag"}
    active_steps = {s for s in _all_steps if getattr(args, s.replace("-", "_"))}
    if not active_steps:
        active_steps = _all_steps

    log(f"執行步驟: {', '.join(sorted(active_steps))}")

    algo_keys = list(ALGO_CONFIGS.keys()) if args.algo == "all" else [args.algo]

    # 預先警告缺少執行檔
    for key in algo_keys:
        cfg = ALGO_CONFIGS[key]
        for bin_name in (cfg["bin_euroc"], cfg["bin_tum"]):
            bin_path = cfg["bin_dir"] / bin_name
            if not bin_path.exists():
                log(f"[WARN] 找不到執行檔: {bin_path}，請先編譯專案")

    results = []

    for algo_key in algo_keys:
        cfg = ALGO_CONFIGS[algo_key]

        # ── EuRoC ──
        if args.dataset in ("euroc", "all"):
            log("\n" + "=" * 65)
            log(f"{cfg['label']} × EuRoC 測試")
            log("=" * 65)
            for seq_folder, ts_stem, seq_suffix in EUROC_DATASETS:
                log(f"\n[{cfg['label']}][EuRoC] {seq_folder}")
                entry = run_euroc(cfg, seq_folder, ts_stem, seq_suffix, active_steps)
                results.append(entry)
                log(f"  → RMSE = {entry['rmse']}  ({entry['note']})")

        # ── TUM-VI ──
        if args.dataset in ("tum", "all"):
            log("\n" + "=" * 65)
            log(f"{cfg['label']} × TUM-VI 測試")
            log("=" * 65)
            for category, seq_name, seq_suffix in TUM_DATASETS:
                log(f"\n[{cfg['label']}][TUM-VI] {seq_name}")
                entry = run_tum(cfg, category, seq_name, seq_suffix, active_steps)
                results.append(entry)
                log(f"  → RMSE = {entry['rmse']}  ({entry['note']})")

    # ── 儲存 CSV ──
    ts_str   = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = SCRIPT_DIR / f"benchmark_results_{ts_str}.csv"

    fieldnames = ["algo", "dataset", "sequence", "output_name", "rmse", "note"]
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)

    # ── 摘要輸出 ──
    log("\n" + "=" * 75)
    log(f"結果已儲存至: {csv_path}")
    log("=" * 75)
    log(f"{'Algo':<18} {'Dataset':<8} {'Sequence':<22} {'RMSE':<14} {'Note'}")
    log("-" * 75)
    for r in results:
        rmse_str = f"{r['rmse']:.6f}" if isinstance(r['rmse'], float) else str(r['rmse'])
        log(f"{r['algo']:<18} {r['dataset']:<8} {r['sequence']:<22} {rmse_str:<14} {r['note']}")


if __name__ == "__main__":
    main()
