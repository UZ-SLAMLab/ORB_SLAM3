# ORB-SLAM3 軌跡研究（純視覺）

## 1. 研究範圍與模式

在 ORB-SLAM3 中，純視覺模式對應 `System::eSensor` 的：

- `MONOCULAR=0`
- `STEREO=1`
- `RGBD=2`

參考：`include/System.h:87-94`

---

## 2. 軌跡的核心資料結構（Tracking 端）

`Tracking` 不會每幀直接輸出完整世界座標軌跡，而是先保存「相對於 reference keyframe 的 frame pose」：

- `mlRelativeFramePoses`：每幀相對其 reference KF 的 SE3 變換 `Tcr`
- `mlpReferences`：每幀對應的 reference keyframe 指標
- `mlFrameTimes`：每幀的時間戳
- `mlbLost`：每幀是否為 lost 狀態（用於輸出時跳過丟失幀）

參考：`include/Tracking.h:154-159`

這四個 list 是最後 `System::SaveTrajectory*()` 重建完整軌跡的來源。

---

## 3. Tracking 狀態機

`Track()` 的行為取決於 `eTrackingState mState`，定義在 `include/Tracking.h:126-133`：

| 狀態               | 值  | 含義                                     |
| :----------------- | :-: | :--------------------------------------- |
| `SYSTEM_NOT_READY` | -1  | 系統尚未就緒                             |
| `NO_IMAGES_YET`    | 0   | 尚未收到任何影像                         |
| `NOT_INITIALIZED`  | 1   | 收到影像但尚未完成初始化                 |
| `OK`               | 2   | 正常追蹤中                               |
| `RECENTLY_LOST`    | 3   | 剛丟失，嘗試重定位（純視覺 3 秒內超時） |
| `LOST`             | 4   | 完全丟失，需要建新地圖                   |

狀態轉移（純視覺）：

```
NO_IMAGES_YET → NOT_INITIALIZED → OK
OK → RECENTLY_LOST（追蹤失敗且 map KF > 10）
OK → LOST（追蹤失敗且 map KF ≤ 10）
RECENTLY_LOST → OK（重定位成功）
RECENTLY_LOST → LOST（超過 3 秒仍無法重定位）
LOST → NO_IMAGES_YET（建立新地圖 / 重設）
```

參考：`src/Tracking.cc:2142-2237`

---

## 4. 前端每幀 pose 估計（純視覺）

主入口：`Tracking::Track()` (`src/Tracking.cc:1963`)

### 4.1 選擇前端路徑的條件

在 `mState == OK` 的正常追蹤下：

```cpp
if ((!mbVelocity && !pCurrentMap->isImuInitialized()) ||
    mCurrentFrame.mnId < mnLastRelocFrameId + 2)
{
    bOK = TrackReferenceKeyFrame();     // tracking_method = 0 (RefKF)
}
else
{
    bOK = TrackWithMotionModel();       // tracking_method = 1 (MM)
    if (!bOK)
        bOK = TrackReferenceKeyFrame(); // fallback to RefKF
}
```

也就是說：
- **沒有速度估計**（首幀 or 上幀丟失後恢復）→ `TrackReferenceKeyFrame`
- **剛做完重定位的前 2 幀** → `TrackReferenceKeyFrame`
- **其他情況** → 先試 `TrackWithMotionModel`，失敗才 fallback

參考：`src/Tracking.cc:2130-2148`

### 4.2 `TrackReferenceKeyFrame()`（BoW 匹配）

步驟：

1. 對當前幀計算 BoW 向量：`mCurrentFrame.ComputeBoW()`
2. 用 `ORBmatcher(0.7, true)` 做 `SearchByBoW(mpReferenceKF, mCurrentFrame, ...)`
   - 匹配門檻：**BoW matches < 15 → 直接失敗**
3. 初始 pose 設為 `mLastFrame.GetPose()`
4. 呼叫 `Optimizer::PoseOptimization(&mCurrentFrame)`
5. 剔除 outlier，統計 `nmatchesMap`（有 >0 次觀測的 inlier 數量）
6. **成功條件：`nmatchesMap >= 10`**（純視覺）；IMU 模式則一律 return true

參考：`src/Tracking.cc:2973-3034`

### 4.3 `TrackWithMotionModel()`（恆速模型）

步驟：

1. **`UpdateLastFrame()`**：
   - 用 `mlRelativeFramePoses.back()` × reference KF 的最新 pose 更新 `mLastFrame` 的姿態
   - 在 localization-only 模式下，還會為 stereo/RGBD 的 last frame 創建臨時 MapPoint（按深度排序，插入深度 < `mThDepth` 的點，至少保留 100 個近點）
   - 參考：`src/Tracking.cc:3036-3108`

2. **初值預測**（純視覺分支）：
   ```cpp
   mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose());
   ```

3. **投影匹配**：用 `ORBmatcher(0.9, true)` 做 `SearchByProjection`
   - 搜索半徑 `th`：stereo=7，其他=15
   - 若匹配數 < 20 → 加倍半徑重試（`2*th`）
   - **仍然 < 20 → 失敗**（純視覺返回 false）

4. 呼叫 `Optimizer::PoseOptimization(&mCurrentFrame)`
5. 剔除 outlier，統計 `nmatchesMap`
6. **成功條件：`nmatchesMap >= 10`**

參考：`src/Tracking.cc:3111-3208`

### 4.4 `mVelocity` 的計算

每幀追蹤成功（`bOK` 或 `RECENTLY_LOST`）後：

```cpp
Sophus::SE3f LastTwc = mLastFrame.GetPose().inverse();
mVelocity = mCurrentFrame.GetPose() * LastTwc;  // Tcl = Tc_curr * Tw_last
mbVelocity = true;
```

即 `mVelocity` = 當前幀 camera-to-world 相對於上一幀的增量，用於下一幀的恆速模型預測。

參考：`src/Tracking.cc:2415-2422`

---

## 5. TrackLocalMap（Local Map 精煉）

`TrackLocalMap()` 是前端追蹤的第二階段，用更多的 local map 點來 refine pose。

### 5.1 建立 Local Map

1. **`UpdateLocalKeyFrames()`**：
   - 當前幀的每個 MapPoint 投票給觀測過它的 KF → 建立 `keyframeCounter`
   - 所有投票到的 KF 加入 `mvpLocalKeyFrames`，同時追蹤得票最高的 `pKFmax` 設為 `mpReferenceKF`
   - 再加入這些 KF 的 covisibility 鄰居（最多 10 個中取 1 個）、子 KF、父 KF
   - 上限 **80 個 local keyframes**
   - 參考：`src/Tracking.cc:3740-3900`

2. **`UpdateLocalPoints()`**：
   - 收集 `mvpLocalKeyFrames` 中所有 KF 的 MapPoint → `mvpLocalMapPoints`
   - 用 `mnTrackReferenceForFrame` 避免重複加入
   - 參考：`src/Tracking.cc:3710-3738`

### 5.2 `SearchLocalPoints()`

1. 已匹配的點標記為已見，不重複搜索
2. 對 `mvpLocalMapPoints` 中未匹配的點做 frustum 檢查（`isInFrustum(pMP, 0.5)`）
3. 用 `ORBmatcher(0.8)` 做 `SearchByProjection`
   - 搜索半徑 `th` 因情況而異：
     - RGBD → th=3
     - 剛重定位 → th=5
     - LOST/RECENTLY_LOST → th=15
     - 預設 → th=1

參考：`src/Tracking.cc:3648-3708`

### 5.3 Pose 優化與成功判定

純視覺條件下（IMU 未初始化），呼叫：

```cpp
Optimizer::PoseOptimization(&mCurrentFrame);   // opt_type = 0
```

優化後統計 `mnMatchesInliers`（非 outlier 且 Observations > 0 的 MapPoint 數量），根據不同情境判斷追蹤是否成功：

| 條件                                     | 門檻             |
| :--------------------------------------- | :--------------- |
| 重定位後 `mMaxFrames` 幀內               | ≥ 50 inliers     |
| `RECENTLY_LOST` 狀態恢復                 | ≥ 10 inliers     |
| 一般純視覺（非 mono/非 IMU）             | **≥ 30 inliers** |
| IMU_MONOCULAR（已初始化）                | ≥ 15 inliers     |
| IMU_STEREO / IMU_RGBD                   | ≥ 15 inliers     |

參考：`src/Tracking.cc:3235-3410`

---

## 6. 純視覺 pose 優化本體

`Optimizer::PoseOptimization()` (`src/Optimizer.cc:814`)

### 6.1 求解器設定

- **線性求解器**：`LinearSolverDense<BlockSolver_6_3>`
- **非線性優化演算法**：Levenberg-Marquardt（`OptimizationAlgorithmLevenberg`）
- **Vertex**：唯一的 `VertexSE3Expmap`（id=0），6 DoF frame pose

### 6.2 邊（觀測）類型

| 觀測類型                             | 邊類型                                   | 維度   |
| :----------------------------------- | :--------------------------------------- | :----- |
| Mono（`mvuRight[i] < 0`）           | `EdgeSE3ProjectXYZOnlyPose`              | 2D     |
| Stereo（`mvuRight[i] >= 0`）        | `EdgeStereoSE3ProjectXYZOnlyPose`        | 3D     |
| 右相機（fisheye 雙目）              | `EdgeSE3ProjectXYZOnlyPoseToBody`        | 2D     |

注意：RGBD 的 `mvuRight` 被填為虛擬右相機 u 值（由深度計算），所以走 **stereo 邊**。

### 6.3 Robust Kernel 與 χ² 門檻

- **Huber Kernel**：
  - mono：`δ = √5.991`（對應 95% χ²(2)）
  - stereo：`δ = √7.815`（對應 95% χ²(3)）
- **Information matrix**：`I × invSigma2[octave]`（根據特徵點的金字塔層級加權）

### 6.4 迭代策略（4 輪）

```cpp
const float chi2Mono[4]   = {5.991, 5.991, 5.991, 5.991};
const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
const int its[4]          = {10, 10, 10, 10};   // 共 40 次迭代
```

每輪結束後：
1. 計算每條邊的 χ² error
2. 超過門檻 → 標記為 outlier，設 `level=1`（下輪不參與優化）
3. 未超過 → 標記為 inlier，設 `level=0`
4. **第 3 輪（it==2）後移除 Robust Kernel**（讓最後一輪用純 L2）
5. 若總邊數 < 10 → 提前終止

**最低門檻**：初始對應數 `nInitialCorrespondences < 3` → 直接返回 0

### 6.5 回傳值

`return nInitialCorrespondences - nBad`：即最終 inlier 數量。同時已將優化後的 pose 寫回 `pFrame->SetPose()`。

參考：`src/Optimizer.cc:828-1114`

---

## 7. 重定位（Relocalization）

當純視覺追蹤進入 `RECENTLY_LOST` 狀態時會嘗試重定位：

### 7.1 候選 KF 檢索

- 對當前幀計算 BoW，用 `mpKeyFrameDB->DetectRelocalizationCandidates()` 查詢候選 KF
- 候選為空 → 直接失敗

### 7.2 MLPnP RANSAC

對每個候選 KF：
1. `ORBmatcher(0.75)` 做 `SearchByBoW`，匹配 < 15 → 丟棄
2. 建立 `MLPnPsolver`，RANSAC 參數：`(0.99, 10, 300, 6, 0.5, 5.991)`
   - 每次跑 5 次 RANSAC 迭代
3. 若得到候選 pose → `PoseOptimization`
   - inlier < 10 → 繼續下一個候選
   - inlier < 50 → 用 `SearchByProjection` 在較寬窗口追加匹配，再次優化
   - inlier ≥ 50 → **重定位成功**

### 7.3 成功後的影響

- 設定 `mnLastRelocFrameId = mCurrentFrame.mnId`
- 之後的 `mMaxFrames` 幀內，TrackLocalMap 的成功門檻提高到 50 inliers
- 前 2 幀強制用 `TrackReferenceKeyFrame`（不走 motion model）

參考：`src/Tracking.cc:3905-4060`

---

## 8. 軌跡保存（reconstruction）

### 8.1 frame 端先存相對 pose

`Track()` 結尾，當 `mState==OK || mState==RECENTLY_LOST` 時：

```cpp
if (mCurrentFrame.isSet()) {
    Sophus::SE3f Tcr_ = mCurrentFrame.GetPose() * mCurrentFrame.mpReferenceKF->GetPoseInverse();
    mlRelativeFramePoses.push_back(Tcr_);
    mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
    mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
    mlbLost.push_back(mState == LOST);
} else {
    // pose 無效 → 複製上一筆（避免 list 長度不一致）
    mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
    mlpReferences.push_back(mlpReferences.back());
    mlFrameTimes.push_back(mlFrameTimes.back());
    mlbLost.push_back(mState == LOST);
}
```

參考：`src/Tracking.cc:2545-2568`

### 8.2 `SaveTrajectoryTUM()` 輸出

1. 取第一個 KF 的 `Twc` 作為原點 offset `Two`
2. 遍歷所有 frame：
   - `*lbL == true`（lost）→ 跳過不輸出
   - 取 reference KF `pKF`；若 `pKF->isBad()` → 沿 spanning tree 的 `mTcp` 往 parent 爬
   - 計算 `Trw = (累積的 Tcp) * pKF->GetPose() * Two`
   - `Tcw = Tcr * Trw`
   - 輸出 `Twc = Tcw.inverse()` 的平移 + 四元數（`x y z qx qy qz qw`）
3. 格式：`timestamp tx ty tz qx qy qz qw`

參考：`src/System.cc:602-651`

### 8.3 `SaveTrajectoryEuRoC()` 輸出

與 TUM 格式的差異：
- 在多地圖（Atlas）中只輸出 **最大的 map**
- 原點 offset：純視覺用 `vpKFs[0]->GetPoseInverse()`；VI 用 `vpKFs[0]->GetImuPose()`
- 若 reference KF 不屬於該 map → 跳過
- 純視覺分支輸出 `Twc`（camera）：
  ```cpp
  Sophus::SE3f Twc = ((*lit) * Trw).inverse();
  ```
- 時間戳以**奈秒**格式輸出（`1e9 * (*lT)`）

參考：`src/System.cc:715-783`（純視覺分支在 `779-783`）

---

## 9. Viewer 中即時軌跡/相機顯示

### 9.1 當前相機 pose 傳遞

Tracking 每幀追蹤完畢後：

```cpp
if (mCurrentFrame.isSet())
    mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());
```

`MapDrawer::SetCurrentCameraPose()` 內部將 `Tcw` 取逆存為 `mCameraPose = Tcw.inverse()`（即 `Twc`）。

參考：`src/Tracking.cc:2403-2407`, `src/MapDrawer.cc:441-445`

### 9.2 `DrawCurrentCamera()`

用 OpenGL 繪製一個梯形線框表示相機視錐體：
- 尺寸由 `Viewer.CameraSize` 控制（`w, h=0.75w, z=0.6w`）
- 顏色：綠色 `(0,1,0)`
- 在 `Twc` 的座標系下用 `glMultMatrix` 繪製

參考：`src/MapDrawer.cc:400-440`

### 9.3 `DrawMapPoints()`

- **一般地圖點**：黑色 `(0,0,0)`
- **Reference map points**（local map 裡的點）：紅色 `(1,0,0)`
- 點大小由 `Viewer.PointSize` 控制

### 9.4 `DrawKeyFrames()`

- 每個 KF 以 `GetPoseInverse()` 取得 `Twc`，在該座標系繪製視錐體線框
- 第一個 KF（無 parent）：紅色、加粗 5 倍線寬
- 其他 KF：藍色
- **Covisibility graph**（綠線）：連接 covisibility weight ≥ 100 的 KF 對
- **Spanning tree**（綠線）：連接每個 KF 與其 parent
- **Loop edges**（綠線）：連接 loop closure 偵測到的 KF 對

參考：`src/MapDrawer.cc:170-400`

---

## 10. 快速判讀：哪些幀是「純視覺」

看 `tracking_log.csv` 時可用以下準則：

- `imu_initialized = no`
- `tracking_method` 不是 `IMU_MM`（值=2）
- `opt_type = 0`（TrackLocalMap 用 `PoseOptimization`）

`tracking_method` 定義（`include/Tracking.h:367-389`）：

| 值 | 含義          |
| :-: | :----------- |
| 0  | RefKF        |
| 1  | MotionModel  |
| 2  | IMU_MM       |
| 3  | Relocalization |

---

## 11. 一句話總結

純視覺軌跡 = 前端以視覺匹配（BoW 或投影）給初值 → `PoseOptimization`（4 輪 × 10 次 Levenberg-Marquardt + Huber outlier 篩選）refine → 每幀只存相對 reference KF 的 `Tcr` → 最後由 `System::SaveTrajectory*()` 沿 spanning tree 回推成完整世界軌跡 `Twc`。
