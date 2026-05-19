# IMU 初始化數學（ORB-SLAM3 Section V.B）

論文來源：J21-ORB_SLAM3.pdf，Section V.B "IMU Initialization"

---

## 時間軸

### t = 0s
Tracking 開始，持續插入 KF，IMU 資料持續預積分。

---

### t ≈ 2s　｜　KF 數量 >= 10　→　**初始化**

- Step 1 完成：純視覺地圖建立（up-to-scale）
- `InitializeIMU()` 第一次呼叫（`priorG = 1e2`）
  - `InertialOptimization()`：估出 $s, \mathbf{R}_{wg}, \mathbf{b}, \bar{\mathbf{v}}$
  - `ApplyScaledRotation()`：scale 和重力對齊套用到地圖
  - `FullInertialBA()`：Visual-Inertial BA
- `isImuInitialized() = true`

---

### t > 5s　→　**VIBA 1**

- `InitializeIMU()` 第二次呼叫（`priorG = 1.f`，放鬆 gyro prior）
  - `InertialOptimization()` + `FullInertialBA()`
- bias 估計開始收斂

---

### t > 15s　→　**VIBA 2**

- `InitializeIMU()` 第三次呼叫（`priorG = 0.f, priorA = 0.f`，完全移除 prior）
  - `InertialOptimization()` + `FullInertialBA()`
- 地圖進入成熟狀態

---

### t = 25 / 35 / 45s　→　**Scale Refinement**（僅 Monocular）

- 條件：KF 數量 <= 200
- `ScaleRefinement()`：每隔 10 秒修正一次 scale
- 注意：程式碼寫了 55 / 65 / 75s 的觸發條件，但因外層有 `mTinit < 50.0f` 擋住，實際上永遠不會執行到

---

### t >= 50s

- 外層條件 `mTinit < 50.0f`（`LocalMapping.cc:235`）不再成立
- 整個初始化流程結束，不再進入

---

## 整體概念

IMU 初始化的目標是為以下惰性變數找到好的初始值，再交給後續的 Visual-Inertial BA：

- **scale** $s$：Monocular 的地圖差一個未知尺度
- **重力方向** $\mathbf{R}_{wg}$：世界座標系對齊重力的旋轉
- **bias** $\mathbf{b} = (\mathbf{b}^a, \mathbf{b}^g) \in \mathbb{R}^6$：加速度計 + 陀螺儀的系統性偏差
- **速度** $\bar{\mathbf{v}}_{0:k}$：各 KF 的初始速度估計

---

## Step 1 — Vision-Only MAP Estimation

先跑純視覺 SLAM 約 2 秒，建出一張 **up-to-scale** 的地圖（k = 10 個 KF）。

得到的位姿為：

$$\bar{\mathbf{T}}_{0:k} = [\mathbf{R},\ \bar{\mathbf{p}}]$$

其中 $\bar{\mathbf{p}}$ 是差一個 scale 的位置（bar 表示 up-to-scale）。

**程式位置**：`LocalMapping.cc:184`，`InitializeIMU()` 觸發之前，LocalMapping 主迴圈在 IMU 尚未初始化時每次都跑 `Optimizer::LocalBundleAdjustment()`，持續約 2 秒所累積的結果就是 Step 1 的輸出。`InitializeIMU()` 進來時直接把這個視覺地圖當作已知輸入。

---

## Step 2 — Inertial-Only MAP Estimation

**對應函式**：`Optimizer::InertialOptimization()`（`Optimizer.cc:3042`）

只用 IMU preintegration 資料，不加視覺 reprojection error。KF 的位姿在這個階段**全部固定不動**（`VP->setFixed(true)`），只優化慣性變數。

### 優化變數 → g2o 頂點對應

$$\mathcal{Y}_k = \{s,\ \mathbf{R}_{wg},\ \mathbf{b},\ \bar{\mathbf{v}}_{0:k}\}$$

| 論文變數 | g2o 頂點 | 說明 |
|----------|----------|------|
| $s$ | `VertexScale` | Stereo 時 `setFixed(true)`，scale 已知 |
| $\mathbf{R}_{wg}$ | `VertexGDir` | 重力方向，在 SO(3) 流形上更新 |
| $\mathbf{b}^g$ | `VertexGyroBias` | 陀螺儀 bias，所有 KF 共用一個 |
| $\mathbf{b}^a$ | `VertexAccBias` | 加速度計 bias，所有 KF 共用一個 |
| $\bar{\mathbf{v}}_i$ | `VertexVelocity`（每個 KF 一個）| 速度 |

### 優化問題（論文公式 8）

$$\mathcal{Y}_k^* = \arg\min_{\mathcal{Y}_k} \left( \|\mathbf{b}\|^2_{\Sigma_b} + \sum_{i=1}^{k} \|\mathbf{r}_{\mathcal{I}_{i-1,i}}\|^2_{\Sigma^{-1}_{\mathcal{I}_{i-1,i}}} \right)$$

### Cost 項 → g2o 邊對應

**第一項**：bias prior（讓 bias 偏向零）

$$\|\mathbf{b}\|^2_{\Sigma_b}$$

```
EdgePriorGyro  →  information = priorG * I₃
EdgePriorAcc   →  information = priorA * I₃
```

prior 強度越大，bias 被拉越靠近零。VIBA 2 時 `priorG = priorA = 0`，代表完全移除此項。

**第二項**：IMU preintegration residual（`EdgeInertialGS`）

$$\|\mathbf{r}_{\mathcal{I}_{i-1,i}}\|^2_{\Sigma^{-1}_{\mathcal{I}_{i-1,i}}}$$

這個 edge 連接 8 個頂點：

```
EdgeInertialGS(T_prev, V_prev, bg, ba, T_curr, V_curr, Rwg, scale)
```

`GS` = Gravity + Scale，代表這條 edge 把重力方向和 scale 也納入優化。這是 Step 2 專屬的 edge 型別。

### IMU Preintegration Residual（論文公式 2）

$$\mathbf{r}_{\mathcal{I}_{i,i+1}} = [\mathbf{r}_{\Delta\mathbf{R}},\ \mathbf{r}_{\Delta\mathbf{v}},\ \mathbf{r}_{\Delta\mathbf{p}}]$$

$$\mathbf{r}_{\Delta\mathbf{R}} = \text{Log}\left(\Delta\mathbf{R}_{i,i+1}^T \mathbf{R}_i^T \mathbf{R}_{i+1}\right)$$

$$\mathbf{r}_{\Delta\mathbf{v}} = \mathbf{R}_i^T\left(\mathbf{v}_{i+1} - \mathbf{v}_i - \mathbf{g}\Delta t\right) - \Delta\mathbf{v}_{i,i+1}$$

$$\mathbf{r}_{\Delta\mathbf{p}} = \mathbf{R}_i^T\left(\mathbf{p}_{i+1} - \mathbf{p}_i - \mathbf{v}_i\Delta t - \frac{1}{2}\mathbf{g}\Delta t^2\right) - \Delta\mathbf{p}_{i,i+1}$$

其中 $\Delta\mathbf{R}, \Delta\mathbf{v}, \Delta\mathbf{p}$ 是 `ImuTypes.cc` 的 `IntegrateNewMeasurement()` 累積出的預積分量。

### 重力方向的更新（論文公式 9）

$\mathbf{R}_{wg}$ 在 SO(3) 流形上更新，但只允許繞非重力軸旋轉：

$$\mathbf{R}_{wg}^{\text{new}} = \mathbf{R}_{wg}^{\text{old}} \cdot \text{Exp}(\delta\alpha_g,\ \delta\beta_g,\ 0)$$

z 軸分量固定為 0，防止重力大小被改動。對應 `VertexGDir` 的 `oplusImpl()`。

### Scale 的更新（論文公式 10）

$$s^{\text{new}} = s^{\text{old}} \cdot \exp(\delta s)$$

指數參數化強制 scale 為正數。對應 `VertexScale` 的 `oplusImpl()`。

---

## Step 3 — Visual-Inertial MAP Estimation

**對應函式**：`Optimizer::FullInertialBA()`（`Optimizer.cc:392`）

有了好的初始值後，把視覺 reprojection error 加回來，做完整的 Visual-Inertial BA。

### 和 Step 2 的關鍵差異

| | Step 2 `InertialOptimization` | Step 3 `FullInertialBA` |
|-|-------------------------------|--------------------------|
| KF 位姿 | **固定不動** | **可優化** |
| scale $s$ | 優化變數（Mono）| 不存在，已套用到地圖 |
| 重力 $\mathbf{R}_{wg}$ | 優化變數 | 不存在，已套用到地圖 |
| bias | 所有 KF 共用一組 | 每個 KF 各自一組（`bInit=false` 時）|
| IMU edge | `EdgeInertialGS`（含 scale、Rwg）| `EdgeInertial`（不含 scale、Rwg）|
| bias 連續性 | 無 | `EdgeGyroRW` + `EdgeAccRW` |
| 視覺項 | 無 | 有 reprojection error |

### 優化問題（論文公式 4）

$$\min_{\mathcal{S}_k,\ \mathcal{X}} \left( \sum_{i=1}^{k} \|\mathbf{r}_{\mathcal{I}_{i-1,i}}\|^2_{\Sigma^{-1}} + \sum_{j=0}^{l-1} \sum_{i \in \mathcal{K}^j} \rho_{\text{Hub}}\left(\|\mathbf{r}_{ij}\|_{\Sigma^{-1}_{ij}}\right) \right)$$

### Cost 項 → g2o 邊對應

**IMU 項**（`EdgeInertial`）：

```
EdgeInertial(T_prev, V_prev, bg_prev, ba_prev, T_curr, V_curr)
```

注意這裡沒有 `Rwg` 和 `scale`，因為這兩個已在 Step 2 後套用到地圖，不再是變數。

**Bias random walk**（論文圖 2(a) 的紫色邊）：

```
EdgeGyroRW(bg_prev, bg_curr)   →  bias 在相鄰 KF 間應平滑變化
EdgeAccRW(ba_prev, ba_curr)
```

**視覺項**：`EdgeMono` / `EdgeStereo`（binary edge，連接 VertexSBAPointXYZ + VertexPose；Huber kernel，閾值 $\sqrt{5.991}$ mono、$\sqrt{7.815}$ stereo）

---

## 整體流程總結

```
LocalMapping::Run()
│
└─ LocalMapping::InitializeIMU()
    │
    ├─ [Step 1 已由 LocalBundleAdjustment 完成] 純視覺地圖，up-to-scale
    │
    ├─ Step 2: Optimizer::InertialOptimization()
    │   ├─ 頂點：VertexScale, VertexGDir, VertexGyroBias, VertexAccBias, VertexVelocity×N
    │   ├─ 邊：EdgeInertialGS×N（含 scale + Rwg）
    │   └─ 邊：EdgePriorGyro + EdgePriorAcc（prior）
    │   → 輸出：s, Rwg, bg, ba, v 的初始估計
    │
    ├─ ApplyScaledRotation()  ← 把 s 和 Rwg 套用到整張地圖，之後不再優化這兩個
    │
    └─ Step 3: Optimizer::FullInertialBA()
        ├─ 頂點：VertexPose×N, VertexVelocity×N
        ├─ 頂點：VertexGyroBias×1, VertexAccBias×1（bInit=true，初始化/VIBA1）
        ├─ 頂點：VertexGyroBias×N, VertexAccBias×N（bInit=false，VIBA2）
        ├─ 邊：EdgeInertial×N（不含 scale + Rwg）
        ├─ 邊：EdgeGyroRW + EdgeAccRW（bInit=false 時才加）
        ├─ 邊：EdgeMono / EdgeStereo reprojection error（Huber kernel）
        └─ 邊：EdgePriorGyro + EdgePriorAcc（bInit=true 時才加）
```

---

## 三階段差異

### 初始化（t ≈ 2s）

**目標**：第一次估出 scale、重力方向、bias、速度的粗略初始值。

- `InertialOptimization`：scale $s$ 和重力方向 $\mathbf{R}_{wg}$ 從頭估起，都是優化變數
- `ApplyScaledRotation()`：把估出的 $s$ 和 $\mathbf{R}_{wg}$ 套用到整張地圖（**三個階段都有執行**）
- `FullInertialBA`（`bInit=true`，因為 priorA ≠ 0）：所有 KF 共用一組 bias，加大 prior（priorG=1e2, priorA=1e5/1e10）把 bias 拉向零，沒有 EdgeGyroRW/AccRW
- `isImuInitialized()` 設為 true

---

### VIBA 1（t > 5s）

**目標**：放鬆 gyro prior，讓 bias 開始收斂到合理值。

- `InertialOptimization`：`mRwg` 起始為 I、`mScale` 起始為 1.0（重力和 scale 已在初始化時套用到地圖，從 neutral 出發），但 `VertexGDir` 和 `VertexScale`（mono）仍是**可優化**的變數，並非固定
- `ApplyScaledRotation()`：同樣執行，把本次 InertialOptimization 的結果套用回地圖
- `FullInertialBA`（`bInit=true`，因為 priorA=1e5 ≠ 0）：仍然共用 bias、仍然有 prior，但 priorG 從 1e2 大幅降到 1.f，讓 gyro bias 自由移動；priorA 維持 1e5
- 和初始化的差別：只有 priorG 改變，gyro 約束大幅放鬆

---

### VIBA 2（t > 15s）

**目標**：完全移除 prior，bias 改用 random walk 模型，讓地圖自由優化到最佳解。

- `InertialOptimization`：同樣執行，priorG=priorA=0 使得 prior 邊的 information=0，實際貢獻為零
- `ApplyScaledRotation()`：同樣執行
- `FullInertialBA`（`bInit=false`，因為 priorA=0）：prior 完全移除，bias 改為每個 KF 各自一組，並加入 EdgeGyroRW/EdgeAccRW 約束 bias 的連續性
- 和 VIBA 1 的差別：`bInit` 從 true 變成 false，bias 模型從「所有 KF 共用常數」升級為「random walk」
- 執行後地圖進入成熟狀態（mature）

---

## 三階段 Prior 強度

| 階段 | 時間 | priorG | priorA | 目的 |
|------|------|--------|--------|------|
| 初始化 | t ≈ 0s | 1e2 | 1e5（stereo）/ 1e10（mono）| 大 prior 穩住解，防止 bias 亂飛 |
| VIBA 1 | t > 5s | 1.f | 1e5 | 放鬆 gyro prior，讓 bias 自由收斂 |
| VIBA 2 | t > 15s | 0.f | 0.f | 完全移除 prior，地圖自己決定 |
| Scale Refinement | t = 25/35/.../75s | — | — | 僅 Mono，純 scale 優化 |
