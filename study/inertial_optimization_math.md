# InertialOptimization() 數學步驟

對應程式：`Optimizer::InertialOptimization()`（`Optimizer.cc:3042`）
Edge 實作：`EdgeInertialGS::computeError()`（`G2oTypes.cc:617`）

---

## 1. 優化變數（頂點）

| 頂點 | 維度 | 說明 |
|------|------|------|
| `VertexPose` × N | 6 | KF 位姿 $\mathbf{T}_i = (\mathbf{R}_{wb_i}, \mathbf{t}_{wb_i})$，**固定不優化** |
| `VertexVelocity` × N | 3 | 各 KF 速度 $\mathbf{v}_i$（world frame） |
| `VertexGyroBias` | 3 | 陀螺儀 bias $\mathbf{b}^g$，所有 KF 共用 |
| `VertexAccBias` | 3 | 加速度計 bias $\mathbf{b}^a$，所有 KF 共用 |
| `VertexGDir` | 2 | 重力方向 $\mathbf{R}_{wg}$（2 自由度，z 軸固定） |
| `VertexScale` | 1 | 尺度 $s$（Stereo 時固定為 1） |

---

## 2. 總 Cost Function

$$\min \left( \underbrace{\|\mathbf{b}^g\|^2_{\text{priorG} \cdot I} + \|\mathbf{b}^a\|^2_{\text{priorA} \cdot I}}_{\text{bias prior}} + \sum_{i=1}^{N} \underbrace{\|\mathbf{r}_{\mathcal{I}_{i-1,i}}\|^2_{\Sigma^{-1}_i}}_{\text{IMU residual}} \right)$$

---

## 3. Bias Prior（`EdgePriorGyro` + `EdgePriorAcc`）

### Residual

$$\mathbf{r}_{\text{prior},g} = \mathbf{0} - \mathbf{b}^g$$

$$\mathbf{r}_{\text{prior},a} = \mathbf{0} - \mathbf{b}^a$$

目標值是零向量，代表「假設 bias 應該接近零」。Cost 取平方後符號不影響結果，但程式碼（`G2oTypes.h:779`）的實際計算是 `bprior - estimate`。

### Information Matrix

$$\Omega_{\text{prior}} = \text{priorG} \cdot I_3 \quad \text{（或 priorA）}$$

`priorG` / `priorA` 越大 → bias 被拉得越靠近零。VIBA 2 時設為 0，代表拿掉此項。

### Jacobian（`G2oTypes.cc:762`）

理論上 $\mathbf{r} = \mathbf{0} - \mathbf{b}$，所以：

$$\frac{\partial \mathbf{r}}{\partial \mathbf{b}} = -I_3$$

但程式碼實際寫 `+I₃`。符號錯誤不影響 Hessian（$J^T\Omega J = \Omega$ 無論正負），只影響 gradient 方向。因為 prior 是弱約束，`EdgeInertialGS` 主導整體優化，實務上不影響收斂。

---

## 4. IMU Preintegration Residual（`EdgeInertialGS`）

每對相鄰 KF $(i-1, i)$ 建一條邊，連接 8 個頂點：

$$(\mathbf{T}_{i-1},\ \mathbf{v}_{i-1},\ \mathbf{b}^g,\ \mathbf{b}^a,\ \mathbf{T}_i,\ \mathbf{v}_i,\ \mathbf{R}_{wg},\ s)$$

### 4.1 預積分量的 bias 修正（`ImuTypes.cc:283`）

預積分量是在某個線性化點 $\bar{\mathbf{b}}$ 計算的。當 bias 變為 $\mathbf{b}$，用一階 Taylor 展開修正，不需重積分：

$$\Delta\mathbf{R}(\mathbf{b}) = \bar{\Delta\mathbf{R}} \cdot \text{Exp}\left(\mathbf{J}^R_g \cdot \delta\mathbf{b}^g\right)$$

$$\Delta\mathbf{v}(\mathbf{b}) = \bar{\Delta\mathbf{v}} + \mathbf{J}^v_g \cdot \delta\mathbf{b}^g + \mathbf{J}^v_a \cdot \delta\mathbf{b}^a$$

$$\Delta\mathbf{p}(\mathbf{b}) = \bar{\Delta\mathbf{p}} + \mathbf{J}^p_g \cdot \delta\mathbf{b}^g + \mathbf{J}^p_a \cdot \delta\mathbf{b}^a$$

其中 $\delta\mathbf{b} = \mathbf{b} - \bar{\mathbf{b}}$，$\mathbf{J}^R_g, \mathbf{J}^v_g, \mathbf{J}^v_a, \mathbf{J}^p_g, \mathbf{J}^p_a$ 是預積分過程中計算好的 Jacobian。

### 4.2 重力向量

$$\mathbf{g} = \mathbf{R}_{wg} \cdot \mathbf{g}_I, \quad \mathbf{g}_I = [0,\ 0,\ -9.81]^T$$

$\mathbf{R}_{wg}$ 把 IMU 定義的重力方向轉到世界座標系。

### 4.3 Residual（`G2oTypes.cc:617`）

**旋轉殘差**（3 維）：

$$\mathbf{r}_{\Delta R} = \text{Log}\!\left(\Delta\mathbf{R}(\mathbf{b})^T \cdot \mathbf{R}_{wb_1}^T \cdot \mathbf{R}_{wb_2}\right)$$

量測值 $\Delta\mathbf{R}$ 應等於 $\mathbf{R}_{wb_1}^T \mathbf{R}_{wb_2}$，差值取 Log 映射回向量空間。

**速度殘差**（3 維）：

$$\mathbf{r}_{\Delta v} = \mathbf{R}_{wb_1}^T \left(s(\mathbf{v}_2 - \mathbf{v}_1) - \mathbf{g}\cdot\Delta t\right) - \Delta\mathbf{v}(\mathbf{b})$$

$s$ 乘在速度差上，因為地圖是 up-to-scale，速度也帶有同一個 scale。

**位置殘差**（3 維）：

$$\mathbf{r}_{\Delta p} = \mathbf{R}_{wb_1}^T \left(s(\mathbf{t}_{wb_2} - \mathbf{t}_{wb_1} - \mathbf{v}_1\Delta t) - \frac{1}{2}\mathbf{g}\Delta t^2\right) - \Delta\mathbf{p}(\mathbf{b})$$

合併成 9 維 residual：

$$\mathbf{r}_{\mathcal{I}} = [\mathbf{r}_{\Delta R},\ \mathbf{r}_{\Delta v},\ \mathbf{r}_{\Delta p}]^T \in \mathbb{R}^9$$

### 4.4 Information Matrix

$$\Omega_i = \Sigma_i^{-1}, \quad \Sigma_i = \mathbf{C}_{[0:9,\ 0:9]}$$

$\mathbf{C}$ 是預積分過程中累積的 $9\times9$ covariance（旋轉、速度、位置），在 `IntegrateNewMeasurement()` 裡遞推計算。

---

## 5. 頂點更新規則

### VertexVelocity / VertexGyroBias / VertexAccBias

加法更新（歐氏空間）：

$$\mathbf{v}^{\text{new}} = \mathbf{v}^{\text{old}} + \delta\mathbf{v}$$

### VertexGDir（`G2oTypes.h:264`）

在 SO(3) 流形上更新，z 軸固定為 0（論文公式 9）：

$$\mathbf{R}_{wg}^{\text{new}} = \mathbf{R}_{wg}^{\text{old}} \cdot \text{Exp}(\delta\alpha,\ \delta\beta,\ 0)$$

只有兩個自由度，防止重力「縮放」（旋轉量的 z 分量鎖死）。

### VertexScale（`G2oTypes.h:314`）

指數更新，強制 scale 為正數（論文公式 10）：

$$s^{\text{new}} = s^{\text{old}} \cdot \exp(\delta s)$$

---

## 6. 優化流程

```
1. 建立頂點：VertexPose（fixed）, VertexVelocity, VertexGyroBias,
             VertexAccBias, VertexGDir, VertexScale

2. 建立邊：
   - EdgePriorGyro（bias^g → 0）
   - EdgePriorAcc（bias^a → 0）
   - EdgeInertialGS × N（每對相鄰 KF 一條）

3. Levenberg-Marquardt 迭代 200 次
   每次迭代：
   a. computeError()：計算所有邊的 residual
   b. linearizeOplus()：計算所有邊對各頂點的 Jacobian
   c. 組裝 H = J^T Ω J，b = J^T Ω r
   d. 解 (H + λI)δx = -b
   e. 用各頂點的 oplusImpl() 更新估計值

4. 讀回優化結果：s, Rwg, bg, ba, v_i
5. 對每個 KF：若 bias 變化量 norm > 0.01 才呼叫 Reintegrate() 重新計算預積分
   否則只呼叫 SetNewBias() 更新 bias 值（Optimizer.cc:3213）
```

---

## 7. 和 FullInertialBA 的差異

| | `InertialOptimization` | `FullInertialBA` |
|-|------------------------|-----------------|
| KF 位姿 | 固定 | 可優化 |
| scale $s$ | 優化變數 | 不存在（已套用到地圖）|
| 重力 $\mathbf{R}_{wg}$ | 優化變數 | 不存在（已套用到地圖）|
| bias | 所有 KF 共用一組 | 每個 KF 各一組 |
| IMU edge | `EdgeInertialGS`（含 $s$, $\mathbf{R}_{wg}$）| `EdgeInertial`（不含）|
| bias 連續性 | 無 | `EdgeGyroRW` + `EdgeAccRW` |
| 視覺項 | 無 | reprojection error |
