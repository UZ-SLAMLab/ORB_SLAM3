#!/usr/bin/env python3
"""
軌跡診斷腳本
=============
將估計軌跡與 GT 對齊後計算每幀 ATE / RPE，並把
tracking_log.csv、localmapping_log.csv、loop_closing_log.csv
對齊到同一條 dataset 時間軸，找出高誤差區段附近發生的
Tracking / LocalMapping / LoopClosing 診斷事件。

使用方式：
  # 最常用：自動尋找 fix_{name}.txt 與三份 log，輸出 PDF + intervals CSV
  python3 evaluation/analyze_trajectory.py \
      --name stereo_imu_euroc_v102 \
      --gt /media/lab405/Windows1/data/Euroc/V1_02_medium/mav0/state_groundtruth_estimate0/data.csv \
      --gt-format euroc

  # 指定 log / output，適合比較不同實驗版本
  python3 evaluation/analyze_trajectory.py \
      --name stereo_imu_euroc_v102_close_lp \
      --gt evaluation/Ground_truth/EuRoC_left_cam/V102_GT.txt \
      --gt-format euroc \
      --traj evaluation/fix_stereo_imu_euroc_v102_close_lp.txt \
      --log evaluation/stereo_imu_euroc_v102_tracking_close_lp_log.csv \
      --local-log evaluation/stereo_imu_euroc_v102_localmapping_close_lp_log.csv \
      --loop-log evaluation/stereo_imu_euroc_v102_loop_closing_close_lp_log.csv \
      --output evaluation/diagnosis_stereo_imu_euroc_v102_close_lp.pdf \
      --interval-output evaluation/diagnosis_stereo_imu_euroc_v102_close_lp_intervals.csv

  # TUM-VI / mocap GT
  python3 evaluation/analyze_trajectory.py \
      --name stereo_imu_tum_out6 \
      --gt /media/lab405/Windows1/data/TUM/outdoors/dataset-outdoors6_512_16/mav0/mocap0/data.csv \
      --gt-format tum

引數：
  --name              output_name；預設對應 evaluation/fix_{name}.txt
                      以及 evaluation/{name}_tracking_log.csv
  --gt                GT 檔案路徑
  --gt-format         euroc 或 tum
  --traj              直接指定估計軌跡路徑（可選）
  --log               直接指定 Tracking log 路徑（可選，自動搜尋）
  --local-log         直接指定 LocalMapping log 路徑（可選，自動搜尋）
  --loop-log          直接指定 LoopClosing log 路徑（可選，自動搜尋）
  --output            診斷 PDF（預設 evaluation/diagnosis_{name}.pdf）
  --interval-output   高誤差區段 CSV
                      （預設和 PDF 同名，加上 _intervals.csv）
  --error-percentile  ATE 高誤差門檻 percentile（預設 90）
  --max-intervals     最多輸出幾段高誤差區段（預設 8）
  --before-window     回查每段高誤差區段前方幾秒（預設 1.0）
  --after-window      回查每段高誤差區段後方幾秒（預設 0.5）
  --no-align          關閉 SE3 對齊（debug 用）

輸出：
  diagnosis_{name}.pdf
    兩頁式診斷圖。每個子圖的 x 軸都是相同的相對 dataset time：
    timestamp - 第一個成功對齊 GT 的估計軌跡 timestamp。

    Page 1: ATE/RPE、tracking inliers、tracking state/method。
    Page 2: visual/IMU residual、bias、backend correction magnitude、
            LocalMapping BA rug marker、LoopClosing/GBA rug marker。Rug marker 是子圖底部短線，
            避免事件太多時把曲線蓋住。
            LocalMapping 的 bias/velocity/pose correction 欄位表示該次
            Local BA local KF window 內的最大 before/after 修正量。

  diagnosis_{name}_intervals.csv
    每段高誤差區間的 start/end/peak time、peak ATE、max RPE、
    dATE/dt，以及 rule-based diagnosis。
"""

import argparse
import csv
import os
import sys
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
os.environ.setdefault("XDG_CACHE_HOME", "/tmp")

import matplotlib
matplotlib.use("Agg")
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.lines import Line2D
import numpy as np
import pandas as pd

SCRIPT_DIR = Path(__file__).parent.resolve()
X_AXIS_LABEL = "Relative dataset time from first matched trajectory frame (s)"
RECENTLY_LOST_COLOR = "#f59e0b"
LOST_COLOR = "#dc2626"
HIGH_ERROR_COLOR = "#fbbf24"
DATE_COLOR = "#0f766e"
LOCAL_BA_COLOR = "#7c3aed"
LOOP_EVENT_COLOR = "#e11d48"
RELOC_COLOR = "#be123c"

# ─────────────────────────────────────────────
#  GT 載入
# ─────────────────────────────────────────────

def load_gt_euroc(gt_path: Path) -> pd.DataFrame:
    """EuRoC GT：timestamp_ns, px, py, pz, qw, qx, qy, qz"""
    rows = []
    with open(gt_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) < 8:
                continue
            ts = float(parts[0]) / 1e9
            px, py, pz = float(parts[1]), float(parts[2]), float(parts[3])
            rows.append({"timestamp": ts, "px": px, "py": py, "pz": pz})
    return pd.DataFrame(rows)


def load_gt_tum(gt_path: Path) -> pd.DataFrame:
    """TUM-VI GT：同 EuRoC 格式（mocap0/data.csv）"""
    return load_gt_euroc(gt_path)


# ─────────────────────────────────────────────
#  估計軌跡載入（TUM 格式）
# ─────────────────────────────────────────────

def load_traj(traj_path: Path) -> pd.DataFrame:
    """TUM 格式：timestamp tx ty tz qx qy qz qw"""
    rows = []
    with open(traj_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            ts = float(parts[0])
            px, py, pz = float(parts[1]), float(parts[2]), float(parts[3])
            rows.append({"timestamp": ts, "px": px, "py": py, "pz": pz})
    return pd.DataFrame(rows)


# ─────────────────────────────────────────────
#  時間戳對齊（最近鄰）
# ─────────────────────────────────────────────

def match_timestamps(est: pd.DataFrame, gt: pd.DataFrame,
                     max_diff: float = 0.05) -> tuple:
    """對每個估計幀找最近的 GT 幀，回傳對齊後的兩個 numpy array (N,3)"""
    est_ts = est["timestamp"].values
    gt_ts  = gt["timestamp"].values
    gt_pos = gt[["px", "py", "pz"]].values

    est_matched, gt_matched, timestamps = [], [], []
    for i, ts in enumerate(est_ts):
        idx = np.argmin(np.abs(gt_ts - ts))
        if np.abs(gt_ts[idx] - ts) < max_diff:
            est_matched.append(est[["px", "py", "pz"]].values[i])
            gt_matched.append(gt_pos[idx])
            timestamps.append(ts)

    return (np.array(timestamps),
            np.array(est_matched),
            np.array(gt_matched))


# ─────────────────────────────────────────────
#  SE3 對齊（Umeyama，無尺度）
# ─────────────────────────────────────────────

def align_se3(est_pos: np.ndarray, gt_pos: np.ndarray):
    """回傳 R (3x3), t (3,)，使 R @ est + t ≈ gt"""
    mu_e = est_pos.mean(axis=0)
    mu_g = gt_pos.mean(axis=0)
    ec = est_pos - mu_e
    gc = gt_pos  - mu_g
    H  = ec.T @ gc
    U, _, Vt = np.linalg.svd(H)
    d  = np.linalg.det(Vt.T @ U.T)
    D  = np.diag([1.0, 1.0, d])
    R  = Vt.T @ D @ U.T
    t  = mu_g - R @ mu_e
    return R, t


# ─────────────────────────────────────────────
#  Log 載入與欄位診斷
# ─────────────────────────────────────────────

def resolve_tracking_log(name: str, explicit_path: str | None) -> Path:
    """自動尋找 Tracking log；支援一般檔名與 close_lp 歷史檔名。"""
    if explicit_path:
        return Path(explicit_path)

    candidates = [SCRIPT_DIR / f"{name}_tracking_log.csv"]
    if name.endswith("_close_lp"):
        base = name[: -len("_close_lp")]
        candidates.append(SCRIPT_DIR / f"{base}_tracking_close_lp_log.csv")
    candidates.append(SCRIPT_DIR / f"{name}_tracking_close_lp_log.csv")

    for p in candidates:
        if p.exists() and p.stat().st_size > 0:
            return p
    return candidates[0]


def resolve_localmapping_log(name: str, explicit_path: str | None) -> Path | None:
    """自動尋找 LocalMapping log。close_lp 的檔名歷史上放在 suffix 前面，這裡一起支援。"""
    if explicit_path:
        p = Path(explicit_path)
        return p if p.exists() else None

    candidates = [SCRIPT_DIR / f"{name}_localmapping_log.csv"]
    if name.endswith("_close_lp"):
        base = name[: -len("_close_lp")]
        candidates.append(SCRIPT_DIR / f"{base}_localmapping_close_lp_log.csv")
    candidates.append(SCRIPT_DIR / f"{name}_localmapping_close_lp_log.csv")

    for p in candidates:
        if p.exists() and p.stat().st_size > 0:
            return p
    return None


def resolve_loopclosing_log(name: str, explicit_path: str | None) -> Path | None:
    if explicit_path:
        p = Path(explicit_path)
        return p if p.exists() and p.stat().st_size > 0 else None

    candidates = [SCRIPT_DIR / f"{name}_loop_closing_log.csv"]
    if name.endswith("_close_lp"):
        base = name[: -len("_close_lp")]
        candidates.append(SCRIPT_DIR / f"{base}_loop_closing_close_lp_log.csv")
    candidates.append(SCRIPT_DIR / f"{name}_loop_closing_close_lp_log.csv")

    for p in candidates:
        if p.exists() and p.stat().st_size > 0:
            return p
    return None


def coerce_numeric_when_possible(df: pd.DataFrame) -> pd.DataFrame:
    for col in df.columns:
        converted = pd.to_numeric(df[col], errors="coerce")
        if converted.notna().any():
            df[col] = converted
    return df


def load_localmapping_log(path: Path) -> pd.DataFrame:
    """
    LocalMapping log 結尾會附上 survive KF 清單，欄位數比 header 少。
    另外舊版 IMU_INITIALIZATION row 曾經多寫一個 -1 placeholder；
    這裡會讀成可修復的表格，避免 pandas 因欄位數不一致直接中止。
    """
    rows = []
    repaired_extra_fields = 0
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = None
        for line_no, row in enumerate(reader, start=1):
            if not row or (row[0].strip().startswith("#")):
                continue
            if header is None:
                header = row
                continue

            while len(row) > len(header) and row[-1] == "":
                row = row[:-1]

            if len(row) == len(header) + 1 and "init_scale" in header and "IMU_INITIALIZATION" in row:
                drop_idx = header.index("init_scale")
                row = row[:drop_idx] + row[drop_idx + 1:]
                repaired_extra_fields += 1

            if len(row) > len(header):
                print(f"[WARN] LocalMapping log line {line_no} has {len(row)} fields, expected {len(header)}; truncating extras.")
                row = row[:len(header)]
            elif len(row) < len(header):
                row = row + [""] * (len(header) - len(row))

            rows.append(row)

    if header is None:
        return pd.DataFrame()
    if repaired_extra_fields:
        print(f"[WARN] Repaired {repaired_extra_fields} LocalMapping IMU_INITIALIZATION row(s) with one extra placeholder field.")

    df = pd.DataFrame(rows, columns=header)
    df = coerce_numeric_when_possible(df)
    if "track_frame_id" in df.columns:
        df["track_frame_id"] = pd.to_numeric(df["track_frame_id"], errors="coerce")
        df = df[df["track_frame_id"].notna()].copy()
        df["track_frame_id"] = df["track_frame_id"].astype(int)
    return df


def load_loopclosing_log(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path, comment="#")
    return coerce_numeric_when_possible(df)


def find_inertial_residual_columns(df: pd.DataFrame) -> list[str]:
    """找真正像 IMU/inertial residual 或 chi2 的欄位，避免把 bias_norm 誤判成 residual。"""
    cols = []
    for col in df.columns:
        c = col.lower()
        has_imu_name = any(k in c for k in ["imu", "inertial", "preint"])
        has_res_name = any(k in c for k in ["residual", "chi2", "error"])
        excluded = any(k in c for k in ["initialized", "predicted", "time", "ms", "bias"])
        if has_imu_name and has_res_name and not excluded:
            cols.append(col)
    return cols


def columns_containing(df: pd.DataFrame, keywords: list[str]) -> list[str]:
    cols = []
    for col in df.columns:
        c = col.lower()
        if any(k in c for k in keywords):
            cols.append(col)
    return cols


def numeric_series(df: pd.DataFrame, col: str) -> pd.Series:
    return pd.to_numeric(df[col], errors="coerce")


def diagnostic_series(df: pd.DataFrame, col: str) -> pd.Series:
    """Diagnostic numeric columns use -1 as 'not collected'; hide it in plots."""
    vals = numeric_series(df, col)
    return vals.mask(vals <= -0.999999)


def first_existing_column(df: pd.DataFrame, candidates: list[str]) -> str | None:
    for col in candidates:
        if col in df.columns:
            return col
    return None


def add_relative_time(df: pd.DataFrame, common_t0: float) -> pd.DataFrame:
    if df is None or df.empty:
        return pd.DataFrame()
    out = df.copy()
    if "timestamp" in out.columns:
        out["_rel_time_s"] = pd.to_numeric(out["timestamp"], errors="coerce") - common_t0
    elif "rel_time_s" in out.columns:
        out["_rel_time_s"] = pd.to_numeric(out["rel_time_s"], errors="coerce")
    else:
        out["_rel_time_s"] = np.nan
    return out


def compute_rpe(est_pos_aligned: np.ndarray, gt_pos: np.ndarray) -> np.ndarray:
    rpe = np.zeros(len(est_pos_aligned), dtype=float)
    if len(est_pos_aligned) > 1:
        est_delta = np.diff(est_pos_aligned, axis=0)
        gt_delta = np.diff(gt_pos, axis=0)
        rpe[1:] = np.linalg.norm(est_delta - gt_delta, axis=1)
    return rpe


def compute_error_derivative(rel_ts: np.ndarray, ate: np.ndarray) -> np.ndarray:
    deriv = np.zeros(len(ate), dtype=float)
    if len(ate) > 1:
        dt = np.diff(rel_ts)
        de = np.diff(ate)
        deriv[1:] = np.divide(de, dt, out=np.zeros_like(de), where=np.abs(dt) > 1e-9)
    return deriv


def find_high_error_intervals(rel_ts: np.ndarray, ate: np.ndarray, rpe: np.ndarray,
                              percentile: float = 90.0,
                              max_intervals: int = 8) -> list[dict]:
    if len(rel_ts) == 0:
        return []
    threshold = float(np.percentile(ate, percentile))
    mask = ate >= threshold
    intervals = []
    start = None
    for i, active in enumerate(mask):
        if active and start is None:
            start = i
        elif not active and start is not None:
            intervals.append((start, i - 1))
            start = None
    if start is not None:
        intervals.append((start, len(mask) - 1))

    rows = []
    for s, e in intervals:
        peak_local = s + int(np.argmax(ate[s:e+1]))
        rows.append({
            "start_time": float(rel_ts[s]),
            "end_time": float(rel_ts[e]),
            "peak_time": float(rel_ts[peak_local]),
            "peak_ATE": float(ate[peak_local]),
            "mean_ATE": float(np.mean(ate[s:e+1])),
            "max_RPE": float(np.max(rpe[s:e+1])),
            "_score": float(ate[peak_local]),
        })
    rows.sort(key=lambda x: x["_score"], reverse=True)
    return rows[:max_intervals]


def window_df(df: pd.DataFrame, start_t: float, end_t: float) -> pd.DataFrame:
    if df is None or df.empty or "_rel_time_s" not in df.columns:
        return pd.DataFrame()
    t = pd.to_numeric(df["_rel_time_s"], errors="coerce")
    return df[(t >= start_t) & (t <= end_t)]


def max_in_window(df: pd.DataFrame, col: str) -> float:
    if df.empty or col not in df.columns:
        return np.nan
    vals = pd.to_numeric(df[col], errors="coerce")
    return float(vals.max()) if vals.notna().any() else np.nan


def max_diag_in_window(df: pd.DataFrame, col: str) -> float:
    if df.empty or col not in df.columns:
        return np.nan
    vals = diagnostic_series(df, col)
    return float(vals.max()) if vals.notna().any() else np.nan


def median_diag(df: pd.DataFrame, col: str) -> float:
    if df.empty or col not in df.columns:
        return np.nan
    vals = diagnostic_series(df, col)
    return float(vals.median()) if vals.notna().any() else np.nan


def min_in_window(df: pd.DataFrame, col: str) -> float:
    if df.empty or col not in df.columns:
        return np.nan
    vals = pd.to_numeric(df[col], errors="coerce")
    return float(vals.min()) if vals.notna().any() else np.nan


def changed_in_window(df: pd.DataFrame, col: str) -> bool:
    if df.empty or col not in df.columns:
        return False
    vals = df[col].dropna().astype(str).unique()
    return len(vals) > 1


def diagnose_interval(interval: dict, tracking_df: pd.DataFrame,
                      lm_df: pd.DataFrame, loop_df: pd.DataFrame,
                      before_window: float = 1.0,
                      after_window: float = 0.5) -> dict:
    start = interval["start_time"] - before_window
    end = interval["end_time"] + after_window
    tw = window_df(tracking_df, start, end)
    lw = window_df(lm_df, start, end)
    cw = window_df(loop_df, start, end)

    reasons = []
    min_inliers = min_in_window(tw, "final_inliers")
    global_median_inliers = pd.to_numeric(tracking_df.get("final_inliers", pd.Series(dtype=float)), errors="coerce").median()
    state_col = "當前幀狀態" if "當前幀狀態" in tracking_df.columns else ("当前帧状态" if "当前帧状态" in tracking_df.columns else "tracking_state")
    state_bad = False
    if state_col in tw.columns:
        state_bad = tw[state_col].astype(str).isin(["LOST", "RECENTLY_LOST"]).any()

    visual_max = max_in_window(tw, "visual_chi2_max")
    visual_global = pd.to_numeric(tracking_df.get("visual_chi2_max", pd.Series(dtype=float)), errors="coerce").median()
    imu_max = max_in_window(tw, "imu_chi2_max")
    imu_global = pd.to_numeric(tracking_df.get("imu_chi2_max", pd.Series(dtype=float)), errors="coerce").median()
    pose_update_max = max_diag_in_window(tw, "pose_update_norm")
    velocity_update_max = max_diag_in_window(tw, "velocity_update_norm")
    bias_update_max = max_diag_in_window(tw, "bias_update_norm")

    if state_bad or (not np.isnan(min_inliers) and not np.isnan(global_median_inliers) and min_inliers < 0.6 * global_median_inliers):
        reasons.append("visual tracking quality dropped")
    if not np.isnan(visual_max) and not np.isnan(visual_global) and visual_global > 0 and visual_max > 2.0 * visual_global:
        reasons.append("visual residual increased")
    if not np.isnan(imu_max) and not np.isnan(imu_global) and imu_global > 0 and imu_max > 2.0 * imu_global:
        reasons.append("IMU residual increased")
    if changed_in_window(tw, "tracking_method"):
        reasons.append("tracking method switched")

    ba_events = []
    imu_init_nearby = False
    if not lw.empty:
        if "event_type" in lw.columns:
            ba_events = lw["event_type"].dropna().astype(str).unique().tolist()
        elif "ba_type" in lw.columns:
            ba_events = lw["ba_type"].dropna().astype(str).unique().tolist()
        if ba_events:
            reasons.append("LocalMapping BA event nearby")
        imu_init_nearby = any("IMU_INITIALIZATION" in ev or "InertialOptimizationInit" in ev for ev in ba_events)
        if not imu_init_nearby and "ba_type" in lw.columns:
            imu_init_nearby = lw["ba_type"].dropna().astype(str).str.contains("InertialOptimizationInit", regex=False).any()
        if imu_init_nearby:
            reasons.append("IMU initialization/refinement event nearby")
        if max_in_window(lw, "imu_chi2_max") > 0:
            reasons.append("LocalMapping inertial residual available")

    lm_visual_before = max_diag_in_window(lw, "visual_chi2_before")
    lm_visual_after = max_diag_in_window(lw, "visual_chi2_after")
    lm_imu_before = max_diag_in_window(lw, "imu_chi2_before")
    lm_imu_after = max_diag_in_window(lw, "imu_chi2_after")
    lm_bias_delta = max_diag_in_window(lw, "bias_delta_norm")
    lm_velocity_delta = max_diag_in_window(lw, "velocity_delta_norm")
    lm_pose_trans = max_diag_in_window(lw, "pose_correction_trans_norm")
    lm_pose_rot = max_diag_in_window(lw, "pose_correction_rot_deg")
    lm_init_scale = max_diag_in_window(lw, "init_scale")
    lm_init_bg_norm = max_diag_in_window(lw, "init_bg_norm")
    lm_init_ba_norm = max_diag_in_window(lw, "init_ba_norm")
    lm_pose_trans_global = median_diag(lm_df, "pose_correction_trans_norm")
    lm_bias_delta_global = median_diag(lm_df, "bias_delta_norm")

    if not np.isnan(lm_pose_trans) and lm_pose_trans > max(0.02, 3.0 * lm_pose_trans_global if not np.isnan(lm_pose_trans_global) else 0.0):
        reasons.append("LocalMapping pose correction was unusually large")
    if not np.isnan(lm_bias_delta) and lm_bias_delta > max(1e-4, 3.0 * lm_bias_delta_global if not np.isnan(lm_bias_delta_global) else 0.0):
        reasons.append("LocalMapping bias correction was unusually large")

    loop_events = []
    if not cw.empty:
        if "event_type" in cw.columns:
            loop_events = cw["event_type"].dropna().astype(str).unique().tolist()
        if loop_events:
            reasons.append("LoopClosing / merge / GBA event nearby")
    loop_sim3_trans = max_diag_in_window(cw, "sim3_correction_trans_norm")
    loop_sim3_rot = max_diag_in_window(cw, "sim3_correction_rot_deg")
    loop_time = max_diag_in_window(cw, "loop_time_ms")
    merge_time = max_diag_in_window(cw, "merge_time_ms")
    gba_time = max_diag_in_window(cw, "gba_time_ms")
    loop_sim3_trans_global = median_diag(loop_df, "sim3_correction_trans_norm")
    if not np.isnan(loop_sim3_trans) and loop_sim3_trans > max(0.02, 3.0 * loop_sim3_trans_global if not np.isnan(loop_sim3_trans_global) else 0.0):
        reasons.append("LoopClosing/MapMerge Sim3 correction was unusually large")

    if not reasons:
        reasons.append("internal logs normal or unavailable; check timestamp/export/evaluation")

    out = dict(interval)
    out.pop("_score", None)
    out.update({
        "window_start": float(start),
        "window_end": float(end),
        "min_final_inliers": min_inliers,
        "max_visual_chi2": visual_max,
        "max_imu_chi2": imu_max,
        "max_pose_update_norm": pose_update_max,
        "max_velocity_update_norm": velocity_update_max,
        "max_bias_update_norm": bias_update_max,
        "tracking_method_changed": changed_in_window(tw, "tracking_method"),
        "bad_tracking_state": bool(state_bad),
        "localmapping_events": ";".join(ba_events),
        "imu_initialization_nearby": bool(imu_init_nearby),
        "max_lm_visual_chi2_before": lm_visual_before,
        "max_lm_visual_chi2_after": lm_visual_after,
        "max_lm_imu_chi2_before": lm_imu_before,
        "max_lm_imu_chi2_after": lm_imu_after,
        "max_lm_bias_delta_norm": lm_bias_delta,
        "max_lm_velocity_delta_norm": lm_velocity_delta,
        "max_lm_pose_correction_trans_norm": lm_pose_trans,
        "max_lm_pose_correction_rot_deg": lm_pose_rot,
        "max_lm_init_scale": lm_init_scale,
        "max_lm_init_bg_norm": lm_init_bg_norm,
        "max_lm_init_ba_norm": lm_init_ba_norm,
        "loopclosing_events": ";".join(loop_events),
        "max_loop_sim3_correction_trans_norm": loop_sim3_trans,
        "max_loop_sim3_correction_rot_deg": loop_sim3_rot,
        "max_loop_time_ms": loop_time,
        "max_merge_time_ms": merge_time,
        "max_gba_time_ms": gba_time,
        "diagnosis": "; ".join(reasons),
    })
    return out


def event_values(df: pd.DataFrame) -> pd.Series:
    if df is None or df.empty:
        return pd.Series(dtype=str)
    if "event_type" in df.columns:
        return df["event_type"].fillna("").astype(str)
    if "ba_type" in df.columns:
        return df["ba_type"].fillna("").astype(str)
    return pd.Series([""] * len(df), index=df.index, dtype=str)


def draw_event_markers(axes, df: pd.DataFrame, color_by_event: dict[str, str],
                       alpha: float = 0.75) -> None:
    """Draw backend events as short bottom rug marks instead of full-height bars."""
    if df is None or df.empty or "_rel_time_s" not in df.columns:
        return
    events = event_values(df)
    times = pd.to_numeric(df["_rel_time_s"], errors="coerce")
    for t_i, ev in zip(times, events):
        if not np.isfinite(t_i):
            continue
        color = color_by_event.get(ev)
        if color is None:
            continue
        for ax in axes:
            ax.vlines(float(t_i), 0.015, 0.09, transform=ax.get_xaxis_transform(),
                      color=color, linewidth=1.0, alpha=alpha)


def shade_intervals(axes, intervals: list[dict]) -> None:
    for interval in intervals:
        s = interval["start_time"]
        e = interval["end_time"]
        for ax in axes:
            ax.axvspan(s, e, color=HIGH_ERROR_COLOR, alpha=0.13, label="_nolegend_")


def shade_tracking_states(axes, times: np.ndarray, states: np.ndarray) -> None:
    if len(times) == 0 or len(states) == 0:
        return

    active_state = None
    active_start = None
    last_t = None
    style = {
        "RECENTLY_LOST": (RECENTLY_LOST_COLOR, 0.14),
        "LOST": (LOST_COLOR, 0.18),
    }

    for t_i, state in zip(times, states):
        if not np.isfinite(t_i):
            continue
        state = str(state)
        state = state if state in style else None
        if state != active_state:
            if active_state is not None and active_start is not None:
                color, alpha = style[active_state]
                for ax in axes:
                    ax.axvspan(active_start, t_i, color=color, alpha=alpha, label="_nolegend_")
            active_state = state
            active_start = float(t_i) if state is not None else None
        last_t = float(t_i)

    if active_state is not None and active_start is not None and last_t is not None:
        color, alpha = style[active_state]
        for ax in axes:
            ax.axvspan(active_start, last_t, color=color, alpha=alpha, label="_nolegend_")


def align_localmapping_to_tracking(lm_df: pd.DataFrame, log_df: pd.DataFrame,
                                   common_t0: float) -> pd.DataFrame:
    """用 LocalMapping 的 track_frame_id 對到 Tracking frame_id，取得 dataset time 軸。"""
    if lm_df.empty or "track_frame_id" not in lm_df.columns or "frame_id" not in log_df.columns:
        return pd.DataFrame()

    frames = log_df[["frame_id", "timestamp"]].copy()
    frames["frame_id"] = pd.to_numeric(frames["frame_id"], errors="coerce")
    frames = frames.dropna(subset=["frame_id"])
    frames["frame_id"] = frames["frame_id"].astype(int)

    lm = lm_df.copy()
    lm["track_frame_id"] = pd.to_numeric(lm["track_frame_id"], errors="coerce")
    lm = lm.dropna(subset=["track_frame_id"])
    lm["track_frame_id"] = lm["track_frame_id"].astype(int)

    aligned = lm.merge(frames, left_on="track_frame_id", right_on="frame_id",
                       how="left", suffixes=("", "_tracking"))
    aligned = aligned.dropna(subset=["timestamp"])
    aligned["rel_time_s"] = aligned["timestamp"] - common_t0
    return aligned


def print_residual_data_report(log_df: pd.DataFrame, lm_df: pd.DataFrame | None,
                               log_imu_res_cols: list[str],
                               lm_imu_res_cols: list[str]) -> None:
    print("\n[Residual data check]")
    if log_imu_res_cols:
        print(f"  Tracking log IMU residual columns: {', '.join(log_imu_res_cols)}")
    else:
        imu_related = columns_containing(log_df, ["imu", "inertial", "preint", "bias"])
        print("  Tracking log has no IMU residual / chi2 columns.")
        print(f"  Tracking log IMU-related columns: {', '.join(imu_related) if imu_related else 'none'}")

    if lm_df is None:
        print("  LocalMapping log not found.")
    elif lm_imu_res_cols:
        print(f"  LocalMapping log IMU residual columns: {', '.join(lm_imu_res_cols)}")
    else:
        reproj_cols = [c for c in ["mean_reprojection_error", "max_reprojection_error"] if c in lm_df.columns]
        print("  LocalMapping log has no IMU residual / chi2 columns.")
        print(f"  LocalMapping available residual-like columns: {', '.join(reproj_cols) if reproj_cols else 'none'}")

    if not log_imu_res_cols and not lm_imu_res_cols:
        print("  Most likely cause: current CSVs do not record EdgeInertial/EdgeInertialGS residuals.")
        print("  tracking_log.csv can show IMU state, bias norm, timing, and optimizer branch,")
        print("  but true inertial residual needs extra C++ logging inside Optimizer/IMU edges.")


# ─────────────────────────────────────────────
#  主程式
# ─────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="軌跡診斷腳本")
    parser.add_argument("--name",      required=True, help="output_name")
    parser.add_argument("--gt",        required=True, help="GT 檔案路徑")
    parser.add_argument("--gt-format", choices=["euroc", "tum"], default="euroc")
    parser.add_argument("--traj",      default=None, help="估計軌跡路徑（可選）")
    parser.add_argument("--log",       default=None, help="tracking log 路徑（可選）")
    parser.add_argument("--local-log", default=None, help="LocalMapping log 路徑（可選，自動搜尋）")
    parser.add_argument("--loop-log",  default=None, help="LoopClosing log 路徑（可選，自動搜尋）")
    parser.add_argument("--output",    default=None, help="輸出圖片檔名")
    parser.add_argument("--interval-output", default=None, help="高誤差區段 CSV 輸出路徑")
    parser.add_argument("--error-percentile", type=float, default=90.0,
                        help="ATE 高誤差區段的 percentile 門檻")
    parser.add_argument("--max-intervals", type=int, default=8,
                        help="最多輸出的高誤差區段數量")
    parser.add_argument("--before-window", type=float, default=1.0,
                        help="回查每段誤差區間前方幾秒")
    parser.add_argument("--after-window", type=float, default=0.5,
                        help="回查每段誤差區間後方幾秒")
    parser.add_argument("--no-align",  action="store_true", help="關閉 SE3 對齊")
    args = parser.parse_args()

    name      = args.name
    traj_path = Path(args.traj) if args.traj else SCRIPT_DIR / f"fix_{name}.txt"
    log_path  = resolve_tracking_log(name, args.log)
    lm_path   = resolve_localmapping_log(name, args.local_log)
    loop_path = resolve_loopclosing_log(name, args.loop_log)
    out_path  = Path(args.output) if args.output else SCRIPT_DIR / f"diagnosis_{name}.pdf"
    interval_out_path = Path(args.interval_output) if args.interval_output else \
        out_path.with_name(f"{out_path.stem}_intervals.csv")
    gt_path   = Path(args.gt)

    # ── 載入資料 ──
    for p, label in [(traj_path, "估計軌跡"), (gt_path, "GT"), (log_path, "Tracking log")]:
        if not p.exists():
            print(f"[ERROR] 找不到 {label}: {p}", file=sys.stderr)
            sys.exit(1)

    print(f"Loading trajectory: {traj_path}")
    est = load_traj(traj_path)

    print(f"Loading GT: {gt_path}")
    gt = load_gt_euroc(gt_path) if args.gt_format == "euroc" else load_gt_tum(gt_path)

    print(f"Loading tracking log: {log_path}")
    log_df = coerce_numeric_when_possible(pd.read_csv(log_path))

    lm_df = None
    if lm_path is not None:
        print(f"Loading LocalMapping log: {lm_path}")
        lm_df = load_localmapping_log(lm_path)
    else:
        print("LocalMapping log: not found (optional)")

    loop_df = None
    if loop_path is not None:
        print(f"Loading LoopClosing log: {loop_path}")
        loop_df = load_loopclosing_log(loop_path)
    else:
        print("LoopClosing log: not found (optional)")

    print(f"Est frames: {len(est)}, GT frames: {len(gt)}, Log frames: {len(log_df)}")
    if lm_df is not None:
        print(f"LocalMapping rows: {len(lm_df)}")
    if loop_df is not None:
        print(f"LoopClosing rows: {len(loop_df)}")

    # ── 時間戳對齊：估計軌跡 ↔ GT ──
    ts_arr, est_pos, gt_pos = match_timestamps(est, gt)
    if len(ts_arr) == 0:
        print("[ERROR] 沒有找到任何對齊的幀，請檢查時間戳格式", file=sys.stderr)
        sys.exit(1)
    print(f"Matched frames: {len(ts_arr)}")

    # ── SE3 對齊 ──
    if not args.no_align:
        R, t = align_se3(est_pos, gt_pos)
        est_pos_aligned = (R @ est_pos.T).T + t
    else:
        est_pos_aligned = est_pos

    # ── 每幀位置誤差 ──
    errors = np.linalg.norm(est_pos_aligned - gt_pos, axis=1)
    rel_ts = ts_arr - ts_arr[0]
    rpe = compute_rpe(est_pos_aligned, gt_pos)
    error_derivative = compute_error_derivative(rel_ts, errors)

    # ── 對齊 tracking log（用軌跡的 t0 當共同參考點）──
    common_t0 = ts_arr[0]          # 軌跡第一幀的絕對時間戳
    tracking_diag_df = add_relative_time(log_df, common_t0)
    log_rel = pd.to_numeric(tracking_diag_df["_rel_time_s"], errors="coerce").values

    lm_aligned = add_relative_time(lm_df, common_t0) if lm_df is not None else pd.DataFrame()
    if lm_df is not None and (lm_aligned.empty or lm_aligned["_rel_time_s"].isna().all()):
        lm_aligned = align_localmapping_to_tracking(lm_df, log_df, common_t0)
        if not lm_aligned.empty:
            lm_aligned["_rel_time_s"] = pd.to_numeric(lm_aligned["rel_time_s"], errors="coerce")
    loop_aligned = add_relative_time(loop_df, common_t0) if loop_df is not None else pd.DataFrame()

    log_imu_res_cols = find_inertial_residual_columns(log_df)
    lm_imu_res_cols = find_inertial_residual_columns(lm_df) if lm_df is not None else []
    print_residual_data_report(log_df, lm_df, log_imu_res_cols, lm_imu_res_cols)

    high_error_intervals = find_high_error_intervals(
        rel_ts, errors, rpe,
        percentile=args.error_percentile,
        max_intervals=args.max_intervals,
    )
    for interval in high_error_intervals:
        mask = (rel_ts >= interval["start_time"]) & (rel_ts <= interval["end_time"])
        interval["max_error_derivative"] = float(np.max(error_derivative[mask])) if mask.any() else np.nan
    diagnosis_rows = [
        diagnose_interval(interval, tracking_diag_df, lm_aligned, loop_aligned,
                          before_window=args.before_window,
                          after_window=args.after_window)
        for interval in high_error_intervals
    ]
    intervals_df = pd.DataFrame(diagnosis_rows)
    if not intervals_df.empty:
        intervals_df.to_csv(interval_out_path, index=False)
        print(f"\nHigh-error intervals saved to: {interval_out_path}")
        print("\n[High-error interval summary]")
        for i, row in intervals_df.iterrows():
            print(f"  #{i+1}: {row['start_time']:.2f}s-{row['end_time']:.2f}s, "
                  f"peak ATE={row['peak_ATE']:.4f}m at {row['peak_time']:.2f}s, "
                  f"max RPE={row['max_RPE']:.4f}m")
            print("      backend: "
                  f"LM pose={row.get('max_lm_pose_correction_trans_norm', np.nan):.4g}m/"
                  f"{row.get('max_lm_pose_correction_rot_deg', np.nan):.4g}deg, "
                  f"LM bias={row.get('max_lm_bias_delta_norm', np.nan):.4g}, "
                  f"init scale={row.get('max_lm_init_scale', np.nan):.4g}, "
                  f"Loop Sim3={row.get('max_loop_sim3_correction_trans_norm', np.nan):.4g}m/"
                  f"{row.get('max_loop_sim3_correction_rot_deg', np.nan):.4g}deg")
            print(f"      diagnosis: {row['diagnosis']}")
    else:
        print("\nHigh-error intervals: none found")

    # ── 軌跡斷點：tracking log 中狀態標記 ──
    state_col_name = first_existing_column(log_df, ["當前幀狀態", "当前帧状态", "tracking_state"])
    if state_col_name:
        state_values = log_df[state_col_name].astype(str).values
        lost_mask = log_df[state_col_name].isin(["LOST", "RECENTLY_LOST"]).values
    else:
        state_values = np.array([""] * len(log_df), dtype=str)
        lost_mask = np.zeros(len(log_df), dtype=bool)
    if "reloc_attempted" in log_df.columns:
        reloc_mask = log_df["reloc_attempted"].astype(str).str.lower().isin(["yes", "true", "1"]).values
    else:
        reloc_mask = np.zeros(len(log_df), dtype=bool)

    # tracking_method 顏色對應
    method_map   = {"none": 0, "RefKF": 1, "MM": 2, "IMU_MM": 3, "Reloc": 4}
    if "tracking_method" in log_df.columns:
        method_vals = log_df["tracking_method"].map(lambda x: method_map.get(str(x), -1)).values
    else:
        method_vals = np.full(len(log_df), -1)

    # ── 繪圖：拆成兩頁，避免每個子圖的 x 軸說明互相擠在一起 ──
    fig1, axes_page1 = plt.subplots(3, 1, figsize=(16, 11.5), sharex=False)
    fig2, axes_page2 = plt.subplots(3, 1, figsize=(16, 11.5), sharex=False)
    fig1.suptitle(f"Trajectory Diagnosis: {name}  |  Page 1/2: Tracking and Error",
                  fontsize=13, fontweight="bold")
    fig2.suptitle(f"Trajectory Diagnosis: {name}  |  Page 2/2: Residuals and Backend Events",
                  fontsize=13, fontweight="bold")
    x_axis_note = (
        "X axis is relative dataset time: timestamp - first matched trajectory timestamp. "
        "Using time lets trajectory error and Tracking/LocalMapping/LoopClosing logs be aligned."
    )
    fig1.text(0.5, 0.935, x_axis_note, ha="center", va="center", fontsize=8)
    fig2.text(0.5, 0.925, x_axis_note, ha="center", va="center", fontsize=8)
    axes = list(axes_page1) + list(axes_page2)

    # 子圖 1：每幀位置誤差
    ax1 = axes[0]
    ax1.plot(rel_ts, errors, color="royalblue", linewidth=0.8, label="ATE (m)")
    ax1.plot(rel_ts, rpe, color="mediumpurple", linewidth=0.7, alpha=0.75, label="RPE-like drift (m)")
    ax1.set_ylabel("ATE / RPE (m)")
    ax1.set_title("Per-frame ATE / RPE (after alignment)")
    ax1.grid(True, alpha=0.3)
    # 標記 RMSE
    rmse = float(np.sqrt(np.mean(errors**2)))
    ax1.axhline(rmse, color="red", linestyle="--", linewidth=0.8,
                label=f"RMSE={rmse:.4f}m")
    ax1_deriv = ax1.twinx()
    ax1_deriv.plot(rel_ts, error_derivative, color=DATE_COLOR, linewidth=0.7,
                   alpha=0.7, label="dATE/dt")
    ax1_deriv.set_ylabel("dATE/dt")
    ax1_deriv.tick_params(axis="y", labelsize=7)
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax1_deriv.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8)

    # 子圖 2：final_inliers vs matches_before_tlm_opt
    ax2 = axes[1]
    if "matches_before_tlm_opt" in log_df.columns:
        ax2.plot(log_rel, numeric_series(log_df, "matches_before_tlm_opt").values,
                 color="orange", linewidth=0.8, linestyle="--", label="before_opt")
    if "final_inliers" in log_df.columns:
        ax2.plot(log_rel, numeric_series(log_df, "final_inliers").values,
                 color="darkgreen", linewidth=0.8, label="final_inliers")
    ax2.set_ylabel("Inliers")
    ax2.set_title("Inliers: before opt (orange--) vs after opt (green)  |  large gap = unstable pose opt")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(True, alpha=0.3)

    # Tracking state background: pale orange=RECENTLY_LOST, pale red=LOST.
    shade_tracking_states([ax1, ax2], log_rel, state_values)

    # Reloc 事件（紅色垂直線）
    for t_i in log_rel[reloc_mask]:
        ax2.axvline(t_i, color=RELOC_COLOR, linewidth=0.7, alpha=0.7)

    # 子圖 3：tracking_method — 用 step 折線顯示，非 MM 一目了然
    ax3 = axes[2]
    ax3.step(log_rel, method_vals, where="mid", color="royalblue", linewidth=0.8)
    ax3.set_yticks(list(method_map.values()))
    ax3.set_yticklabels(list(method_map.keys()), fontsize=8)
    ax3.set_ylim(-0.5, max(method_map.values()) + 0.5)

    # RECENTLY_LOST / LOST 狀態背景
    if state_col_name:
        rl_mask = log_df[state_col_name].astype(str) == "RECENTLY_LOST"
        lo_mask = log_df[state_col_name].astype(str) == "LOST"
        in_rl = False; in_lo = False; rl_s = lo_s = None
        for t_i, is_rl, is_lo in zip(log_rel, rl_mask.values, lo_mask.values):
            if is_rl and not in_rl:
                rl_s = t_i; in_rl = True
            elif not is_rl and in_rl:
                ax3.axvspan(rl_s, t_i, alpha=0.18, color=RECENTLY_LOST_COLOR); in_rl = False
            if is_lo and not in_lo:
                lo_s = t_i; in_lo = True
            elif not is_lo and in_lo:
                ax3.axvspan(lo_s, t_i, alpha=0.22, color=LOST_COLOR); in_lo = False
        if in_rl and rl_s is not None:
            ax3.axvspan(rl_s, np.nanmax(log_rel), alpha=0.18, color=RECENTLY_LOST_COLOR)
        if in_lo and lo_s is not None:
            ax3.axvspan(lo_s, np.nanmax(log_rel), alpha=0.22, color=LOST_COLOR)

    ax3.set_ylabel("Tracking Method")
    ax3.set_title("Tracking Method (step)  |  orange=RECENTLY_LOST  red=LOST")
    ax3.legend(handles=[
        mpatches.Patch(color=RECENTLY_LOST_COLOR, alpha=0.25, label="RECENTLY_LOST"),
        mpatches.Patch(color=LOST_COLOR, alpha=0.28, label="LOST"),
    ], loc="upper right", fontsize=7)
    ax3.grid(True, alpha=0.3)

    # 子圖 4：慣性 residual；若目前 CSV 沒記，明確畫出可用替代資訊
    ax4 = axes[3]
    if log_imu_res_cols:
        plotted_residual = False
        for col in log_imu_res_cols:
            vals = diagnostic_series(log_df, col)
            if vals.notna().any():
                ax4.plot(log_rel, vals.values, linewidth=0.8, label=col)
                plotted_residual = True
        ax4.set_ylabel("IMU residual")
        if plotted_residual:
            ax4.set_title("Recorded IMU/Inertial Residuals from Tracking Log")
            ax4.legend(loc="upper right", fontsize=8)
        else:
            ax4.set_title("IMU Residual: columns exist, but values are not collected")
            ax4.text(0.5, 0.5, "IMU residual columns are present, but all values are -1/NaN.",
                     ha="center", va="center", transform=ax4.transAxes, fontsize=10)
    elif lm_imu_res_cols and not lm_aligned.empty:
        plotted_residual = False
        for col in lm_imu_res_cols:
            vals = diagnostic_series(lm_aligned, col)
            if vals.notna().any():
                ax4.plot(lm_aligned["_rel_time_s"].values, vals.values,
                         linewidth=0.8, marker=".", markersize=2, label=col)
                plotted_residual = True
        ax4.set_ylabel("IMU residual")
        if plotted_residual:
            ax4.set_title("Recorded IMU/Inertial Residuals from LocalMapping Log")
            ax4.legend(loc="upper right", fontsize=8)
        else:
            ax4.set_title("IMU Residual: LocalMapping values are not collected")
            ax4.text(0.5, 0.5, "LocalMapping IMU residual columns are present, but all values are -1/NaN.",
                     ha="center", va="center", transform=ax4.transAxes, fontsize=10)
    elif not lm_aligned.empty and {"mean_reprojection_error", "max_reprojection_error"}.issubset(lm_aligned.columns):
        ax4.plot(lm_aligned["_rel_time_s"].values, numeric_series(lm_aligned, "mean_reprojection_error").values,
                 color="purple", linewidth=0.8, marker=".", markersize=2, label="mean_reprojection_error")
        ax4.plot(lm_aligned["_rel_time_s"].values, numeric_series(lm_aligned, "max_reprojection_error").values,
                 color="brown", linewidth=0.6, alpha=0.55, label="max_reprojection_error")
        ax4.set_ylabel("Reprojection error")
        ax4.set_title("No true IMU residual in current CSVs; showing LocalMapping reprojection error proxy")
        ax4.text(0.01, 0.92,
                 "True inertial residual requires logging EdgeInertial/EdgeInertialGS chi2/residual in C++.",
                 transform=ax4.transAxes, fontsize=8, va="top",
                 bbox=dict(facecolor="white", alpha=0.75, edgecolor="none"))
        ax4.legend(loc="upper right", fontsize=8)
    else:
        ax4.text(0.5, 0.5,
                 "No IMU residual columns found in tracking/localmapping logs.\n"
                 "Current tracking_log.csv only records IMU state, bias norm, timing, and optimizer branch.\n"
                 "To plot true inertial residual, add C++ logging around EdgeInertial / EdgeInertialGS chi2.",
                 ha="center", va="center", transform=ax4.transAxes, fontsize=10)
        ax4.set_ylabel("IMU residual")
        ax4.set_title("IMU Residual: data not recorded")
    ax4.grid(True, alpha=0.3)

    # 子圖 5：bias norm（VI 才有意義，純視覺全為 0）
    ax5 = axes[4]
    if "bias_acc_norm" in log_df.columns:
        ax5.plot(log_rel, numeric_series(log_df, "bias_acc_norm").values,
                 color="tomato", linewidth=0.8, label="bias_acc_norm")
    if "bias_gyro_norm" in log_df.columns:
        ax5.plot(log_rel, numeric_series(log_df, "bias_gyro_norm").values,
                 color="steelblue", linewidth=0.8, label="bias_gyro_norm")
    if "bias_update_norm" in log_df.columns:
        vals = diagnostic_series(log_df, "bias_update_norm")
        if vals.notna().any():
            ax5.plot(log_rel, vals.values,
                     color="#111827", linewidth=0.65, alpha=0.75, label="bias_update_norm")
    if "bias_delta_norm" in lm_aligned.columns:
        vals = diagnostic_series(lm_aligned, "bias_delta_norm")
        if vals.notna().any():
            ax5.plot(lm_aligned["_rel_time_s"].values, vals.values,
                     color=LOCAL_BA_COLOR, linewidth=0.0, marker=".", markersize=3,
                     label="LM bias_delta_norm")
    ax5.set_ylabel("Bias norm")
    ax5.set_xlabel(X_AXIS_LABEL)
    ax5.set_title("IMU Bias Norm (always 0 in pure visual mode)")
    ax5.legend(loc="upper right", fontsize=8)
    ax5.grid(True, alpha=0.3)

    # 子圖 6：後端修正量。這些值只描述 BA / loop correction 前後差異，不代表又跑了一次優化。
    ax6 = axes[5]
    plotted_backend = False
    if not lm_aligned.empty and "pose_correction_trans_norm" in lm_aligned.columns:
        vals = diagnostic_series(lm_aligned, "pose_correction_trans_norm")
        if vals.notna().any():
            ax6.plot(lm_aligned["_rel_time_s"].values, vals.values,
                     color=LOCAL_BA_COLOR, linewidth=0.0, marker=".", markersize=3,
                     label="LM pose correction trans (m)")
            plotted_backend = True
    if not lm_aligned.empty and "velocity_delta_norm" in lm_aligned.columns:
        vals = diagnostic_series(lm_aligned, "velocity_delta_norm")
        if vals.notna().any():
            ax6.plot(lm_aligned["_rel_time_s"].values, vals.values,
                     color="#0891b2", linewidth=0.0, marker=".", markersize=2.5,
                     label="LM velocity delta norm")
            plotted_backend = True
    if not loop_aligned.empty and "sim3_correction_trans_norm" in loop_aligned.columns:
        vals = diagnostic_series(loop_aligned, "sim3_correction_trans_norm")
        if vals.notna().any():
            ax6.plot(loop_aligned["_rel_time_s"].values, vals.values,
                     color=LOOP_EVENT_COLOR, linewidth=0.0, marker="x", markersize=4,
                     label="Loop Sim3 trans (m)")
            plotted_backend = True
    if not lm_aligned.empty and "init_scale" in lm_aligned.columns:
        vals = diagnostic_series(lm_aligned, "init_scale")
        if vals.notna().any():
            ax6.plot(lm_aligned["_rel_time_s"].values, vals.values,
                     color="#16a34a", linewidth=0.0, marker="D", markersize=3.5,
                     label="IMU init scale")
            plotted_backend = True

    ax6_rot = ax6.twinx()
    plotted_rot = False
    if not lm_aligned.empty and "pose_correction_rot_deg" in lm_aligned.columns:
        vals = diagnostic_series(lm_aligned, "pose_correction_rot_deg")
        if vals.notna().any():
            ax6_rot.plot(lm_aligned["_rel_time_s"].values, vals.values,
                         color="#4338ca", linewidth=0.0, marker=".", markersize=2.5,
                         label="LM pose correction rot (deg)")
            plotted_rot = True
    if not loop_aligned.empty and "sim3_correction_rot_deg" in loop_aligned.columns:
        vals = diagnostic_series(loop_aligned, "sim3_correction_rot_deg")
        if vals.notna().any():
            ax6_rot.plot(loop_aligned["_rel_time_s"].values, vals.values,
                         color=DATE_COLOR, linewidth=0.0, marker="^", markersize=3.5,
                         label="Loop Sim3 rot (deg)")
            plotted_rot = True

    ax6.set_ylabel("Translation / norm")
    ax6_rot.set_ylabel("Rotation (deg)")
    ax6.set_title("Backend Correction Magnitude  |  large spikes may explain trajectory jumps")
    if plotted_backend or plotted_rot:
        lines_l, labels_l = ax6.get_legend_handles_labels()
        lines_r, labels_r = ax6_rot.get_legend_handles_labels()
        ax6.legend(lines_l + lines_r, labels_l + labels_r, loc="upper right", fontsize=8)
    else:
        ax6.text(0.5, 0.5,
                 "Backend correction columns are missing or all -1/NaN in current logs.",
                 ha="center", va="center", transform=ax6.transAxes, fontsize=10)
    ax6.grid(True, alpha=0.3)

    # ── 高誤差區段與後端事件標記 ──
    shade_intervals(axes, high_error_intervals)
    draw_event_markers(axes_page2, lm_aligned, {
        "LOCAL_BA": LOCAL_BA_COLOR,
        "LOCAL_INERTIAL_BA": LOCAL_BA_COLOR,
        "IMU_INITIALIZATION": "#111827",
        "InertialOptimizationInit": "#111827",
    })
    draw_event_markers(axes_page2, loop_aligned, {
        "LOOP_DETECTED": LOOP_EVENT_COLOR,
        "LOOP_CLOSED": LOOP_EVENT_COLOR,
        "LOOP_CLOSURE": LOOP_EVENT_COLOR,
        "MAP_MERGE": "saddlebrown",
        "GBA_STARTED": "teal",
        "GBA_FINISHED": "darkcyan",
    })

    # ── 對齊 x 軸範圍 ──
    t_max_log = float(np.nanmax(log_rel)) if len(log_rel) and np.isfinite(log_rel).any() else 0.0
    t_max_err = float(rel_ts[-1])
    t_max_lm = float(lm_aligned["_rel_time_s"].max()) if not lm_aligned.empty else 0.0
    t_max_loop = float(loop_aligned["_rel_time_s"].max()) if not loop_aligned.empty else 0.0
    for ax in axes:
        ax.set_xlim(0, max(t_max_log, t_max_err, t_max_lm, t_max_loop))
        ax.set_xlabel(X_AXIS_LABEL)

    # ── 圖例補充 ──
    recently_lost_patch = mpatches.Patch(color=RECENTLY_LOST_COLOR, alpha=0.25,
                                          label="RECENTLY_LOST interval")
    lost_patch = mpatches.Patch(color=LOST_COLOR, alpha=0.28, label="LOST interval")
    reloc_line = Line2D([0], [0], color=RELOC_COLOR, lw=1.0, label="Reloc attempted")
    high_err_patch = mpatches.Patch(color=HIGH_ERROR_COLOR, alpha=0.28,
                                    label="High-error interval")
    lm_marker = Line2D([0], [0], color=LOCAL_BA_COLOR, marker="|", markersize=12,
                       linestyle="None", label="LocalMapping BA rug")
    lc_marker = Line2D([0], [0], color=LOOP_EVENT_COLOR, marker="|", markersize=12,
                       linestyle="None", label="LoopClosing / GBA rug")
    legend_handles = [recently_lost_patch, lost_patch, reloc_line,
                      high_err_patch, lm_marker, lc_marker]
    for fig_i in [fig1, fig2]:
        fig_i.legend(handles=legend_handles,
                     loc="lower center", ncol=6, fontsize=8, framealpha=0.8)
        fig_i.tight_layout(rect=[0, 0.06, 1, 0.91])

    if out_path.suffix.lower() == ".pdf":
        with PdfPages(out_path) as pdf:
            pdf.savefig(fig1, dpi=150, bbox_inches="tight")
            pdf.savefig(fig2, dpi=150, bbox_inches="tight")
        print(f"\nDiagnosis PDF saved to: {out_path} (2 pages)")
    else:
        page2_path = out_path.with_name(f"{out_path.stem}_page2{out_path.suffix or '.png'}")
        fig1.savefig(out_path, dpi=150, bbox_inches="tight")
        fig2.savefig(page2_path, dpi=150, bbox_inches="tight")
        print(f"\nDiagnosis plots saved to: {out_path} and {page2_path}")
    plt.close(fig1)
    plt.close(fig2)
    if intervals_df.empty:
        pd.DataFrame(columns=[
            "start_time", "end_time", "peak_time", "peak_ATE", "mean_ATE",
            "max_RPE", "max_error_derivative",
            "max_pose_update_norm", "max_velocity_update_norm", "max_bias_update_norm",
            "max_lm_visual_chi2_before", "max_lm_visual_chi2_after",
            "max_lm_imu_chi2_before", "max_lm_imu_chi2_after",
            "max_lm_bias_delta_norm", "max_lm_velocity_delta_norm",
            "max_lm_pose_correction_trans_norm", "max_lm_pose_correction_rot_deg",
            "imu_initialization_nearby", "max_lm_init_scale",
            "max_lm_init_bg_norm", "max_lm_init_ba_norm",
            "max_loop_sim3_correction_trans_norm", "max_loop_sim3_correction_rot_deg",
            "max_loop_time_ms", "max_merge_time_ms", "max_gba_time_ms",
            "diagnosis"
        ]).to_csv(interval_out_path, index=False)
        print(f"High-error intervals saved to: {interval_out_path} (empty)")
    print(f"RMSE = {rmse:.6f} m  (matched frames: {len(ts_arr)})")


if __name__ == "__main__":
    main()
