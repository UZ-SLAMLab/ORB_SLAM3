# ORB-SLAM3 軌跡研究（視覺 + 慣性）

## 1. 研究範圍與模式

視覺慣性模式對應 `System::eSensor`：

- `IMU_MONOCULAR=3`
- `IMU_STEREO=4`
- `IMU_RGBD=5`

參考：`include/System.h:87-94`

---

## 2. 整體骨架（先有共同流程，再進入 VI 分支）

與純視覺相同，軌跡最終仍由這組資料結構重建：

- `mlRelativeFramePoses`：每幀相對 reference KF 的 `Tcr`
- `mlpReferences`：每幀的 reference keyframe 指標
- `mlFrameTimes`：每幀時間戳
- `mlbLost`：每幀是否為 lost

參考：`include/Tracking.h:154-159`

也就是說：VI 並不是換一套軌跡保存容器，而是改變「每幀 pose 是怎麼估計出來」。

---

## 3. VI 前端估計：何時用 IMU

主入口仍是 `Tracking::Track()` (`src/Tracking.cc:1963`)。

### 3.1 IMU 預積分準備

每幀進入追蹤前，若 sensor 為 VI 且非剛建立新地圖，會做 `PreintegrateIMU()`：

```cpp
if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
    && !mbCreatedMap)
{
    PreintegrateIMU();
}
```

`PreintegrateIMU()` 做兩件事：
1. 從 `mlQueueImuData` 取出上一幀到當前幀之間的所有 IMU 量測
2. 把它們積入 `mpImuPreintegratedFromLastKF`（KF→當前幀）和 `mCurrentFrame.mpImuPreintegratedFrame`（上一幀→當前幀）
3. 積分完後呼叫 `mCurrentFrame.setIntegrated()`

參考：`src/Tracking.cc:2046-2055`, `src/Tracking.cc:1825-1903`

### 3.2 Motion model 分支中的 IMU 條件

在 `TrackWithMotionModel()`：

```cpp
if (mpAtlas->isImuInitialized() && (mCurrentFrame.mnId > mnLastRelocFrameId + mnFramesToResetIMU))
{
    PredictStateIMU();
    return true;      // 直接成功，不做視覺投影匹配
}
```

若滿足：
- map 已 `isImuInitialized()`
- frame id 已超過重定位後 reset 視窗（`mnFramesToResetIMU`）

就直接用 IMU 預測 pose 並返回 true（`tracking_method = IMU_MM = 2`），不走後面那段純視覺投影匹配初始化。

參考：`src/Tracking.cc:3119-3126`

### 3.3 `PredictStateIMU()` 實際做什麼

用上次狀態 + 重力 + IMU 預積分，預測 `(Rwb, twb, Vwb)`，再轉成 frame 的 `Tcw`。

**map 已更新時**（`mbMapUpdated && mpLastKeyFrame`）→ 參照 last **keyframe**：

```cpp
const Eigen::Vector3f twb1 = mpLastKeyFrame->GetImuPosition();
const Eigen::Matrix3f Rwb1 = mpLastKeyFrame->GetImuRotation();
const Eigen::Vector3f Vwb1 = mpLastKeyFrame->GetVelocity();
const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
const float t12 = mpImuPreintegratedFromLastKF->dT;

Rwb2 = NormalizeRotation(Rwb1 * ΔR(bias));
twb2 = twb1 + Vwb1*t12 + 0.5*t12²*Gz + Rwb1 * Δp(bias);
Vwb2 = Vwb1 + t12*Gz + Rwb1 * Δv(bias);
```

**map 未更新時**（`!mbMapUpdated`）→ 參照 last **frame**：
- 使用 `mCurrentFrame.mpImuPreintegratedFrame`（只覆蓋上一幀到當前幀）
- 公式同上但參考物改為 `mLastFrame`

最後呼叫 `mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2)` 把 body state 轉成 `Tcw`：

```cpp
Sophus::SE3f Twb(Rwb, twb);
mTcw = mImuCalib.mTcb * Twb.inverse();   // Tcb * Tbw = Tcw
```

參考：`src/Tracking.cc:1915-1949`, `src/Frame.cc:483-492`

### 3.4 RECENTLY_LOST 下的 IMU 行為

當 `mState == RECENTLY_LOST` 時：
- **VI 模式**：若 IMU 已初始化 → 用 `PredictStateIMU()` 繼續預測（不做重定位）
  - 若超過 `time_recently_lost` 秒仍無法恢復 → 轉為 LOST
  - 若 IMU 未初始化 → 直接 `bOK = false`
- **純視覺模式**：嘗試 `Relocalization()`（3 秒超時）

這意味著 VI 模式可以在短暫丟失時純靠 IMU 慣性維持軌跡連續性。

參考：`src/Tracking.cc:2190-2220`

---

## 4. VI 下的 pose refine（關鍵）

`TrackLocalMap()` 會依條件切 3 種優化：

### 4.1 選擇邏輯

```cpp
if (!mpAtlas->isImuInitialized())
{
    Optimizer::PoseOptimization(&mCurrentFrame);           // opt_type = 0
}
else
{
    if (mCurrentFrame.mnId <= mnLastRelocFrameId + mnFramesToResetIMU)
    {
        Optimizer::PoseOptimization(&mCurrentFrame);       // opt_type = 0（重定位 reset 視窗內）
    }
    else
    {
        if (!mbMapUpdated)
            inliers = Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame);   // opt_type = 1
        else
            inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame); // opt_type = 2
    }
}
```

| opt_type | 函式                                     | 觸發條件                               |
| :------: | :--------------------------------------- | :------------------------------------- |
| 0        | `PoseOptimization`                       | IMU 未初始化，或重定位 reset 視窗內    |
| 1        | `PoseInertialOptimizationLastFrame`      | IMU 已初始化 + 視窗外 + map 未更新     |
| 2        | `PoseInertialOptimizationLastKeyFrame`   | IMU 已初始化 + 視窗外 + map 已更新     |

參考：`src/Tracking.cc:3235-3261`

### 4.2 成功判定（VI 模式）

| 感測器模式           | 門檻                                                  |
| :------------------- | :---------------------------------------------------- |
| IMU_MONOCULAR（已初始化） | < 15 inliers → 失敗；< 50（未初始化）→ 失敗         |
| IMU_STEREO / IMU_RGBD    | < 15 inliers → 失敗                                 |
| 重定位後 mMaxFrames 幀內 | < 50 inliers → 失敗                                 |
| RECENTLY_LOST 恢復       | ≥ 10 inliers → 成功                                 |

參考：`src/Tracking.cc:3370-3410`

---

## 5. VI 優化圖模型長什麼樣

### 5.1 `PoseInertialOptimizationLastKeyFrame`

參考：`src/Optimizer.cc:4491-4735`

**求解器**：`BlockSolverX` + `LinearSolverDense` + **Gauss-Newton**（非 Levenberg-Marquardt）

**Vertex（8 個）**：

| id | 類型               | 優化/固定 | 說明                |
| :-: | :---------------- | :-------- | :------------------ |
| 0  | `VertexPose`       | 優化      | 當前 frame pose     |
| 1  | `VertexVelocity`   | 優化      | 當前 frame 速度     |
| 2  | `VertexGyroBias`   | 優化      | 當前 frame 陀螺偏置 |
| 3  | `VertexAccBias`    | 優化      | 當前 frame 加速偏置 |
| 4  | `VertexPose`       | **固定**  | 上一個 KF pose      |
| 5  | `VertexVelocity`   | **固定**  | 上一個 KF 速度      |
| 6  | `VertexGyroBias`   | **固定**  | 上一個 KF 陀螺偏置  |
| 7  | `VertexAccBias`    | **固定**  | 上一個 KF 加速偏置  |

**Edge（慣性部分）**：

| 邊類型             | 連接                                   | Information                          |
| :----------------- | :------------------------------------- | :----------------------------------- |
| `EdgeInertial`     | VPk→VVk→VGk→VAk→VP→VV               | 預積分噪聲協方差的逆                 |
| `EdgeGyroRW`       | VGk→VG                                | `C[9:12,9:12]⁻¹`（陀螺隨機遊走）   |
| `EdgeAccRW`        | VAk→VA                                | `C[12:15,12:15]⁻¹`（加速隨機遊走） |

**Edge（視覺部分）**：
- `EdgeMonoOnlyPose`（mono 觀測，2D）
- `EdgeStereoOnlyPose`（stereo 觀測，3D）
- Huber robust kernel：mono `δ=√5.991`，stereo `δ=√7.815`
- Information matrix 考慮了 `mpCamera->uncertainty2(obs)` 的校正

**迭代策略**：4 輪 × 10 次，χ² 門檻逐輪收緊：

```cpp
float chi2Mono[4]   = {12,   7.5,  5.991, 5.991};
float chi2Stereo[4] = {15.6, 9.8,  7.815, 7.815};
```

且距離 < 10m 的 close point 門檻放寬 1.5 倍（`chi2close = 1.5 * chi2Mono[it]`）。

第 3 輪（it==2）後移除 Robust Kernel。

### 5.2 `PoseInertialOptimizationLastFrame`

參考：`src/Optimizer.cc:5036-5108`

與 LastKeyFrame 版本的**關鍵差異**：

1. **Previous vertex 不固定**：上一個 **frame**（非 KF）的 pose/velocity/bias 也被優化（id=4~7，`setFixed(false)`）
2. **使用 frame-to-frame 預積分**：`pFrame->mpImuPreintegratedFrame`（而非 KF→frame）
3. **額外加入先驗邊**：
   ```cpp
   EdgePriorPoseImu* ep = new EdgePriorPoseImu(pFp->mpcpi);
   ```
   - 連接上一個 frame 的 4 個 vertex
   - 用 Huber kernel `δ=5`
   - `mpcpi` 是 previous frame 保存的 constrained prior info（`ConstrainedPoseImu`），包含已優化的 state 與 information matrix

4. **χ² 門檻**：mono 不逐輪收緊（4 輪都是 `5.991`），stereo 逐輪收緊（`{15.6, 9.8, 7.815, 7.815}`）

這讓連續幀下的 VI 解更穩定，因為有上一幀的先驗約束。

### 5.3 兩個 VI 優化的回傳值

回傳 `nInitialCorrespondences - nBad`（inlier 數量），同時已更新 frame 的：
- pose（`VP->estimate()`）
- velocity（`VV->estimate()`）
- gyro bias（`VG->estimate()`）
- acc bias（`VA->estimate()`）

若 inlier < 30 且非 `bRecInit`，會用放寬門檻（mono=18, stereo=24）再做一次 outlier 判定。

---

## 6. IMU 初始化對軌跡的影響（LocalMapping）

`LocalMapping` 會負責 IMU 初始化與慣性 BA 階段切換。

### 6.1 初始化觸發條件

```cpp
if (!mpCurrentKeyFrame->GetMap()->isImuInitialized() && mbInertial)
{
    if (mbMonocular)
        InitializeIMU(1e2, 1e10, true);   // mono：先驗極弱
    else
        InitializeIMU(1e2, 1e5, true);    // stereo/RGBD：先驗較強
}
```

參考：`src/LocalMapping.cc:214-221`

### 6.2 `InitializeIMU()` 流程

1. **前置條件**：KF 數量 ≥ `nMinKF`（10），且時間跨度 ≥ `minTime`（mono=2s, stereo=1s）
2. **估計重力方向 `Rwg`**：
   - 累積所有 KF 的 `R_prev * ΔV` → 得到重力方向 `dirG`
   - 用 `gI=(0,0,-1)` 與 `dirG` 的叉積算旋轉 `Rwg = SO3::exp(vzg)`
3. **估計初始速度**：`v_k = (p_{k+1} - p_k) / ΔT`
4. **`InertialOptimization`**：聯合優化 `Rwg`、scale `s`、gyro bias `bg`、acc bias `ba`
5. **若 `scale < 0.1` → 初始化失敗，直接返回**
6. **套用尺度與旋轉**：
   ```cpp
   Sophus::SE3f Twg(mRwg.transpose(), Vector3f::Zero());
   mpAtlas->GetCurrentMap()->ApplyScaledRotation(Twg, mScale, true);
   mpTracker->UpdateFrameIMU(mScale, vpKF[0]->GetImuBias(), mpCurrentKeyFrame);
   ```
7. **標記完成**：`mpAtlas->SetImuInitialized()`
8. **FullInertialBA**：做完整的慣性 BA（100 次迭代）

參考：`src/LocalMapping.cc:1268-1405`

### 6.3 局部 BA 的分支

IMU 初始化後，`LocalMapping` 的局部 BA 從 `LocalBundleAdjustment` 切換為 `LocalInertialBA`：

```cpp
if (mbInertial && mpCurrentKeyFrame->GetMap()->isImuInitialized())
{
    Optimizer::LocalInertialBA(mpCurrentKeyFrame, &mbAbortBA, pMap, ...);
}
else
{
    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, pMap, ...);
}
```

參考：`src/LocalMapping.cc:156-177`

---

## 7. `UpdateFrameIMU()` 為什麼和軌跡研究有關

`UpdateFrameIMU(s, b, pCurrentKeyFrame)` 在 IMU 初始化/重估尺度後被呼叫，它做三件事：

### 7.1 歷史 frame 相對位姿重標定

```cpp
for (auto lit = mlRelativeFramePoses.begin(); ...; lit++)
{
    if (*lbL) continue;           // 跳過 lost 幀
    KeyFrame* pKF = *lRit;
    while (pKF->isBad()) pKF = pKF->GetParent();
    if (pKF->GetMap() == pMap)
        (*lit).translation() *= s;   // 只縮放平移部分
}
```

這代表：VI 初始化/重估尺度後，歷史 frame 相對位姿會被重標定，而不是只改當前幀。

### 7.2 更新 bias

```cpp
mLastBias = b;
mLastFrame.SetNewBias(mLastBias);
mCurrentFrame.SetNewBias(mLastBias);
```

### 7.3 重設當前/上一幀的 IMU pose/velocity

- 若 `mLastFrame` 就是 last KF 同一幀 → 直接用 KF 的 IMU state
- 否則 → 從 last KF 的 state 透過預積分推算
- `mCurrentFrame` 也用相同方式從其 `mpLastKeyFrame` 推算

參考：`src/Tracking.cc:4268-4332`

---

## 8. frame pose 與 body pose 的關係

`Frame::SetImuPoseVelocity(Rwb, twb, Vwb)` 會做：

```cpp
mVw = Vwb;
Sophus::SE3f Twb(Rwb, twb);
Sophus::SE3f Tbw = Twb.inverse();
mTcw = mImuCalib.mTcb * Tbw;    // Tcb * Tbw = Tcw
```

參考：`src/Frame.cc:483-492`

因此 tracking 內主狀態仍是 `Tcw`（相機位姿），但它可由 IMU 狀態透過 extrinsic `Tcb` 推導而來。反過來，要從 `Tcw` 回推 body pose：

```cpp
Twb = (Tcb * Tcw)^-1 = Twc * Tbc
```

---

## 9. 軌跡輸出：VI 與純視覺的關鍵差異

### 9.1 共同點

輸出時都會用：

- `mlRelativeFramePoses` + reference KF + spanning tree 回推

參考：`src/System.cc:715-767`, `820-872`

### 9.2 `SaveTrajectoryEuRoC()` 差異

**原點 offset**：
```cpp
if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
    Twb = vpKFs[0]->GetImuPose();     // body frame 原點
else
    Twb = vpKFs[0]->GetPoseInverse();  // camera frame 原點
```

**VI 模式輸出 `Twb`（body）**：
```cpp
Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
// Tbc * Tcr * Trw = Tbw → 取逆 = Twb
f << 1e9*(*lT) << " " << twb << " " << q << endl;
```

**純視覺輸出 `Twc`（camera）**：
```cpp
Sophus::SE3f Twc = ((*lit) * Trw).inverse();
// Tcr * Trw = Tcw → 取逆 = Twc
```

參考：`src/System.cc:770-783`

也就是說，研究 VI 軌跡時要先確認你看的檔案是 body 還是 camera 參考座標。

### 9.3 `SaveTrajectoryTUM()` 的情況

`SaveTrajectoryTUM()` **不區分 VI/純視覺**，一律輸出 `Twc`（camera pose），不做 body 轉換。所以若想比較 body 軌跡，應使用 EuRoC 格式。

參考：`src/System.cc:602-651`

---

## 10. 怎麼判斷某一幀主要走 VI 還是純視覺

看 `tracking_log.csv`（欄位定義在 `include/Tracking.h:356-419`）：

1. `imu_initialized = yes` 才可能走慣性分支
2. `tracking_method`：
   - `0` = RefKF（純視覺）
   - `1` = MotionModel（純視覺恆速模型）
   - `2` = IMU_MM（IMU 預測，**不做視覺投影匹配初始化**）
   - `3` = Relocalization
3. `opt_type`：
   - `0` = `PoseOptimization`（純視覺優化）
   - `1` = `PoseInertialOptimizationLastFrame`（VI，參照上一 frame）
   - `2` = `PoseInertialOptimizationLastKeyFrame`（VI，參照上一 KF）
4. `imu_predicted`：是否有用 IMU 做初始 pose 預測

典型 VI 幀的 log 特徵：`tracking_method=2, imu_initialized=yes, opt_type=1 或 2, imu_predicted=true`

---

## 11. VI 模式下的特殊機制

### 11.1 Bad IMU 偵測

若 `LocalMapping` 偵測到 IMU 異常（`mbBadImu = true`），`Track()` 開頭會直接重設地圖：

```cpp
if (mpLocalMapper->mbBadImu) {
    mpSystem->ResetActiveMap();
    return;
}
```

參考：`src/Tracking.cc:1980-1984`

### 11.2 時間戳跳躍處理

若當前幀時間戳比上一幀大超過 1 秒：
- IMU 已初始化且完成 InertialBA2 → `CreateMapInAtlas()`（建新地圖）
- IMU 已初始化但未完成 → `ResetActiveMap()`
- IMU 未初始化 → `ResetActiveMap()`

參考：`src/Tracking.cc:2010-2040`

### 11.3 重定位後的 IMU reset 視窗

重定位成功後的 `mnFramesToResetIMU` 幀內：
- 前端仍使用純視覺 `PoseOptimization`（不走 VI 優化）
- 前 2 幀強制用 `TrackReferenceKeyFrame`
- 視窗結束時呼叫 `ResetFrameIMU()` 重置 IMU 狀態

參考：`src/Tracking.cc:2370-2395`

---

## 12. 一句話總結

VI 軌跡不是「視覺與 IMU 二選一」，而是以視覺觀測為量測主體，IMU 透過預積分與偏置狀態把時間連續性和動力學先驗注入同一個 pose 優化裡；初始化時還會回溯校正歷史 frame 的尺度。
