import matplotlib.pyplot as plt
import os
import copy
from matplotlib.backends.backend_pdf import PdfPages
from evo.tools import file_interface
from evo.core import sync # 新增：用來對齊時間戳

# --- 設定：請確保這兩個檔案跟這個 .py 檔案在同一個資料夾 ---
# 注意：EuRoC 的 GT 通常是這個 data.csv 檔案
gt_file = '/home/lab405/henry/for_git/ORB_SLAM3/data/Euroc/V1_02_medium/mav0/state_groundtruth_estimate0/data.csv' 
# 注意：ORB-SLAM3 跑完 EuRoC 預設產生的檔名通常是 CameraTrajectory.txt (TUM 格式)
pred_file = 'fix_stereo_euroc_v102.txt' 

# --- 輸出設定 ---
save_pdf = True
pred_stem = os.path.splitext(os.path.basename(pred_file))[0]
pdf_output = f"{pred_stem}.pdf"


# --- 1. 讀取數據 (加入錯誤檢查，防止檔案找不到) ---
if not os.path.exists(gt_file) or not os.path.exists(pred_file):
    print(f"錯誤：找不到數據檔。請確保 '{gt_file}' 和 '{pred_file}' 都在這個資料夾裡。")
    exit()

print("正在讀取數據...")

# 修改：使用 evo 讀取 EuRoC CSV 格式與 TUM 格式
traj_ref = file_interface.read_euroc_csv_trajectory(gt_file)
traj_est = file_interface.read_tum_trajectory_file(pred_file)

print("正在同步時間戳並進行 SE(3) 對齊...")

# 新增：同步軌跡的時間戳 (將估計軌跡與 GT 對齊，允許最大時間差 0.01 秒)
max_diff = 0.01
traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est, max_diff)

# 複製一份估計軌跡來進行對齊
traj_est_aligned = copy.deepcopy(traj_est)

# 執行 SE(3) 對齊 (如果你跑的是單目 Monocular，缺乏真實尺度，請把 correct_scale 改為 True)
traj_est_aligned.align(traj_ref, correct_scale=False)

# 修改：提取 X 軸與 Y 軸數據 (EuRoC 俯視圖通常看 X 與 Y)
# evo 將資料整理為 Nx3 的矩陣，索引 0 是 X，索引 1 是 Y，索引 2 是 Z
gt_x = traj_ref.positions_xyz[:, 0]
gt_y = traj_ref.positions_xyz[:, 1]

pred_x = traj_est_aligned.positions_xyz[:, 0]
pred_y = traj_est_aligned.positions_xyz[:, 1]

# --- 2. 開始畫圖 ---
plt.style.use('seaborn-v0_8-white')
fig = plt.figure(figsize=(10, 8), facecolor='white')
ax = fig.add_subplot(111)

print("正在畫圖")

# 畫出真實軌跡
ax.plot(gt_x, gt_y, color='black', linestyle='-', linewidth=2.5, label='Ground Truth')

# 估計軌跡：用時間漸層著色（藍→紅，對應開始→結束）
import numpy as np
n = len(pred_x)
colors = plt.cm.rainbow(np.linspace(0, 1, max(n - 1, 1)))
for i in range(n - 1):
    ax.plot(pred_x[i:i+2], pred_y[i:i+2], color=colors[i], linewidth=2.0)

# 標記起點與終點
ax.plot(pred_x[0],  pred_y[0],  'go', markersize=8, label=f'Start  t=0s')
ax.plot(pred_x[-1], pred_y[-1], 'rs', markersize=8, label=f'End')

# 每隔固定幀數標一個時間戳（估計軌跡的 timestamp）
ts = traj_est_aligned.timestamps
t0 = ts[0]
interval = max(1, n // 10)   # 最多標 10 個點
for i in range(0, n, interval):
    ax.annotate(f'{ts[i]-t0:.1f}s', (pred_x[i], pred_y[i]),
                fontsize=7, color='dimgray',
                xytext=(4, 4), textcoords='offset points')

# colorbar 顯示時間進度
sm = plt.cm.ScalarMappable(cmap='rainbow',
                            norm=plt.Normalize(vmin=0, vmax=ts[-1]-t0))
sm.set_array([])
plt.colorbar(sm, ax=ax, label='Elapsed time (s)', shrink=0.7)

# --- 3. 美化與強制等比例 ---
ax.axis('equal')
ax.set_xlabel('x [m]', fontsize=14, fontweight='bold')
ax.set_ylabel('y [m]', fontsize=14, fontweight='bold')
ax.set_title('Stereo SLAM Trajectory Evaluation (SE3) — color = time', fontsize=14, fontweight='bold', pad=15)
ax.tick_params(axis='both', which='major', labelsize=12)
ax.legend(loc='upper right', fontsize=11, frameon=True, edgecolor='black')
ax.grid(False)

# 三維立體軌跡圖
# 1. 提取 Z 軸數據 (索引 2 代表 Z 軸)
gt_z = traj_ref.positions_xyz[:, 2]
pred_z = traj_est_aligned.positions_xyz[:, 2]

# 2. 建立第二個畫布與 3D 座標軸
fig_3d = plt.figure(figsize=(10, 8), facecolor='white')
ax_3d = fig_3d.add_subplot(111, projection='3d')

# 3. 畫出 3D 軌跡線條
ax_3d.plot(gt_x, gt_y, gt_z, color='black', linestyle='-', linewidth=2, label='Ground Truth')

# 估計軌跡：時間漸層著色
n3 = len(pred_x)
colors3 = plt.cm.rainbow(np.linspace(0, 1, max(n3 - 1, 1)))
for i in range(n3 - 1):
    ax_3d.plot(pred_x[i:i+2], pred_y[i:i+2], pred_z[i:i+2],
               color=colors3[i], linewidth=2.0)

# 起點與終點標記
ax_3d.scatter(pred_x[0],  pred_y[0],  pred_z[0],  c='green', s=60, zorder=5, label='Start')
ax_3d.scatter(pred_x[-1], pred_y[-1], pred_z[-1], c='red',   s=60, zorder=5, marker='s', label='End')

# 時間標記
interval3 = max(1, n3 // 10)
for i in range(0, n3, interval3):
    ax_3d.text(pred_x[i], pred_y[i], pred_z[i],
               f'{ts[i]-t0:.1f}s', fontsize=7, color='dimgray')

# 4. 3D 圖表美化與標籤
ax_3d.set_xlabel('x [m]', fontsize=12, fontweight='bold')
ax_3d.set_ylabel('y [m]', fontsize=12, fontweight='bold')
ax_3d.set_zlabel('z [m]', fontsize=12, fontweight='bold')
ax_3d.set_title('3D Trajectory (SE3) — color = time', fontsize=14, fontweight='bold', pad=15)
ax_3d.legend(loc='upper right', fontsize=11, frameon=True, edgecolor='black')

# 5. 強制 3D 空間等比例 (避免某個軸被過度拉長，適用於 Matplotlib 3.3.0 以上)
try:
    ax_3d.set_box_aspect((1, 1, 1))
except AttributeError:
    pass  # 若 matplotlib 版本較舊則略過等比例設定

# 6. 互動式點擊顯示時間：在軌跡上疊加可點擊的 scatter
_pick_sc = ax_3d.scatter(pred_x, pred_y, pred_z,
                          c=np.linspace(0, 1, n3), cmap='rainbow',
                          s=8, alpha=0.0, picker=5)   # alpha=0 不顯示，picker=5 點擊容差

_annot_text = ax_3d.text2D(0.02, 0.95, '', transform=ax_3d.transAxes,
                             fontsize=10, color='black',
                             bbox=dict(boxstyle='round,pad=0.3', fc='yellow', alpha=0.8))

def _on_pick(event):
    if event.artist is not _pick_sc:
        return
    idx = event.ind[0]
    t_sec = ts[idx] - t0
    _annot_text.set_text(f't = {t_sec:.2f}s  (idx={idx})')
    fig_3d.canvas.draw_idle()

fig_3d.canvas.mpl_connect('pick_event', _on_pick)

# XZ 平面與 YZ 平面軌跡圖
# 確保已經提取 Z 軸數據 (如果前面 3D 圖已經有這兩行也沒關係，重複賦值不影響)
gt_z = traj_ref.positions_xyz[:, 2]
pred_z = traj_est_aligned.positions_xyz[:, 2]

# 畫出 XZ 平面 (正視圖)
fig_xz = plt.figure(figsize=(10, 8), facecolor='white')
ax_xz = fig_xz.add_subplot(111)

ax_xz.plot(gt_x, gt_z, color='black', linestyle='-', linewidth=2.5, label='Ground Truth')
ax_xz.plot(pred_x, pred_z, color='red', linestyle='-', linewidth=2.5, label='ORB-SLAM3')

ax_xz.axis('equal')
ax_xz.set_xlabel('x [m]', fontsize=14, fontweight='bold')
ax_xz.set_ylabel('z [m]', fontsize=14, fontweight='bold')
ax_xz.set_title('Trajectory Evaluation - XZ Plane (Front View)', fontsize=16, fontweight='bold', pad=15)
ax_xz.tick_params(axis='both', which='major', labelsize=12)
ax_xz.legend(loc='upper right', fontsize=12, frameon=True, edgecolor='black')
ax_xz.grid(False)

# 畫出 YZ 平面 (側視圖)
fig_yz = plt.figure(figsize=(10, 8), facecolor='white')
ax_yz = fig_yz.add_subplot(111)

ax_yz.plot(gt_y, gt_z, color='black', linestyle='-', linewidth=2.5, label='Ground Truth')
ax_yz.plot(pred_y, pred_z, color='red', linestyle='-', linewidth=2.5, label='ORB-SLAM3')

ax_yz.axis('equal')
ax_yz.set_xlabel('y [m]', fontsize=14, fontweight='bold')
ax_yz.set_ylabel('z [m]', fontsize=14, fontweight='bold')
ax_yz.set_title('Trajectory Evaluation - YZ Plane (Side View)', fontsize=16, fontweight='bold', pad=15)
ax_yz.tick_params(axis='both', which='major', labelsize=12)
ax_yz.legend(loc='upper right', fontsize=12, frameon=True, edgecolor='black')
ax_yz.grid(False)


# --- 4. 輸出 PDF ---
if save_pdf:
    with PdfPages(pdf_output) as pdf:
        pdf.savefig(fig, bbox_inches='tight')
        pdf.savefig(fig_3d, bbox_inches='tight')
        pdf.savefig(fig_xz, bbox_inches='tight')
        pdf.savefig(fig_yz, bbox_inches='tight')
    print(f"已儲存 PDF：{pdf_output}")

print("請查看彈出的圖片視窗。")
plt.show()