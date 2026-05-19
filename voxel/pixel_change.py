import open3d as o3d
import numpy as np
import os
from pathlib import Path                                                                                                                                                                                                                                                
                                                                                                                                                                                                                                                                          
  # 必須要有這一行！                                                                                                                                                                                                                                                      
SCRIPT_DIR = Path(__file__).parent.resolve()
# ==========================================
# 檔案路徑設定 (對應你的電腦環境)
# ==========================================
INPUT_PLY = str(SCRIPT_DIR / "stereo_imu_tum_room1_mappoints.txt")  # 這是你從 ORB-SLAM3 產生的點雲檔案                                                                                                                                                                                                      
OUTPUT_TXT = str(SCRIPT_DIR / "stereo_imu_tum_room1_mappoints_fix.txt")                                                                                                                                                                                                      
                                                                                                                                                                                                                                                                          
print(f"Reading from: {INPUT_PLY}") 
print(f"Writing to: {OUTPUT_TXT}")


# Voxel 的大小 (方塊大小)
# 如果跑出來方塊太小或太大，可以調整這個數字 (例如 0.05 或 0.1)
VOXEL_SIZE = 0.05

# 測試模式：True = 所有點一律設為白色 (255,255,255)，忽略原始灰階值
FORCE_WHITE = True

def main():
    if not os.path.exists(INPUT_PLY):
        print(f"找不到點雲檔案：{INPUT_PLY}")
        return

    print(f"正在讀取點雲：{INPUT_PLY} ...")

    points_list = []
    colors_list = []
    with open(INPUT_PLY, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            vals = [float(v) for v in line.replace(',', ' ').split()]
            if len(vals) < 3:
                continue
            points_list.append(vals[:3])
            if len(vals) >= 6:
                # XYZ + RGB (0-255)
                colors_list.append([vals[3]/255.0, vals[4]/255.0, vals[5]/255.0])
            elif len(vals) == 4:
                # XYZ + Grayscale (0-255)
                g = 1.0 if FORCE_WHITE else vals[3] / 255.0
                colors_list.append([g, g, g])
            else:
                colors_list.append([1.0, 1.0, 1.0])

    if not points_list:
        print("找不到任何有效點，請確認檔案格式。")
        return

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points_list))
    pcd.colors = o3d.utility.Vector3dVector(np.array(colors_list))
    print(f"原始點雲共有 {len(pcd.points)} 個點。")

    # ==========================================
    # 核心：將點雲「體素化 (Voxel Downsampling)」
    # ==========================================
    print(f"正在進行 Voxel 轉換 (Voxel Size = {VOXEL_SIZE})...")
    voxel_pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
    print(f"體素化完成！剩下 {len(voxel_pcd.points)} 個 Voxel。")

    points = np.asarray(voxel_pcd.points)
    
    # ORB-SLAM2 原本沒有存顏色，所以我們預設給它白色 (255, 255, 255)
    # 如果未來你有帶顏色的點雲，這段程式碼也會自動去抓顏色
    if len(voxel_pcd.colors) > 0:
        colors = np.asarray(voxel_pcd.colors) * 255.0
    else:
        colors = np.ones_like(points) * 255.0

    # ==========================================
    # 寫出符合你 C++ 程式碼讀取的 TXT 格式
    # ==========================================
    print(f"正在寫入 TXT 檔案：{OUTPUT_TXT} ...")
    with open(OUTPUT_TXT, 'w') as f:
        for i in range(len(points)):
            x, y, z = points[i]
            r, g, b = colors[i]
            # 你的 C++ 需要的格式：X Y Z R G B (用空白隔開)
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\n")

    print("現在可以去跑你的 C++ OpenGL 程式了！")

if __name__ == "__main__":
    main()