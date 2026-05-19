# FullInertialBA() 數學步驟

對應程式：`Optimizer::FullInertialBA()`（`Optimizer.cc:392`）
Edge 實作：`EdgeInertial::computeError()`（`G2oTypes.cc:514`）

---

## 1. 兩種呼叫模式（`bInit` 參數）

`FullInertialBA` 在初始化和正常 BA 時行為不同：

| | `bInit = true` | `bInit = false` |
|-|----------------|-----------------|
| bias 頂點 | 所有 KF 共用一組 | 每個 KF 各一組 |
| EdgeGyroRW / EdgeAccRW | 無 | 有 |
| bias prior | 有（`EdgePriorGyro` + `EdgePriorAcc`）| 無 |

`bInit` 的值由呼叫端的 `priorA` 決定（`LocalMapping.cc:1403`）：

```cpp
if (priorA != 0.f)
    FullInertialBA(..., bInit=true,  priorG, priorA);  // 初始化、VIBA 1
else
    FullInertialBA(..., bInit=false);                   // VIBA 2
```

| 階段 | priorA | bInit |
|------|--------|-------|
| 初始化（t ≈ 2s） | 1e5 / 1e10 | true |
| VIBA 1（t > 5s） | 1e5 | true |
| VIBA 2（t > 15s）| 0 | **false** |

VIBA 2 用 `bInit=false`，代表每個 KF 有各自的 bias，並加上 `EdgeGyroRW`/`EdgeAccRW` 約束 bias 連續性，不加 prior。

---

## 2. 優化變數（頂點）

### bInit = true 時

| 頂點 | 維度 | 說明 |
|------|------|------|
| `VertexPose` × N | 6 | KF 位姿 $(\mathbf{R}_{wb_i}, \mathbf{t}_{wb_i})$，**可優化** |
| `VertexVelocity` × N | 3 | 各 KF 速度 $\mathbf{v}_i$ |
| `VertexGyroBias` × 1 | 3 | 陀螺儀 bias $\mathbf{b}^g$，所有 KF 共用 |
| `VertexAccBias` × 1 | 3 | 加速度計 bias $\mathbf{b}^a$，所有 KF 共用 |
| `VertexSBAPointXYZ` × M | 3 | MapPoint 3D 位置（marginalized）|

**注意**：沒有 `VertexGDir`、`VertexScale`，因為 scale 和重力對齊在 `InertialOptimization` 後已套用到地圖，不再是優化變數。

---

## 3. 總 Cost Function

$$\min \left(
\underbrace{\sum_{i=1}^{N} \|\mathbf{r}_{\mathcal{I}_{i-1,i}}\|^2_{\Sigma^{-1}_i}}_{\text{IMU}} +
\underbrace{\sum_{j} \rho_{\text{Hub}}\!\left(\|\mathbf{r}_{\text{reproj},j}\|^2_{\Sigma^{-1}_j}\right)}_{\text{視覺}} +
\underbrace{\|\mathbf{0} - \mathbf{b}^g\|^2_{\text{priorG}\cdot I} + \|\mathbf{0} - \mathbf{b}^a\|^2_{\text{priorA}\cdot I}}_{\text{bias prior（bInit=true 才有）}}
\right)$$

---

## 4. IMU Preintegration Residual（`EdgeInertial`）

每對相鄰 KF 建一條邊，連接 **6 個頂點**（比 `EdgeInertialGS` 少 scale 和 Rwg）：

$$(\mathbf{T}_{i-1},\ \mathbf{v}_{i-1},\ \mathbf{b}^g_{i-1},\ \mathbf{b}^a_{i-1},\ \mathbf{T}_i,\ \mathbf{v}_i)$$

重力向量 $\mathbf{g}$ 在這裡是**固定常數**（已在 `InertialOptimization` 估好）：

$$\mathbf{g} = [0,\ 0,\ -9.81]^T \quad \text{（world frame，z 軸向上）}$$

### 4.1 預積分量的 bias 修正（與 EdgeInertialGS 相同）

$$\Delta\mathbf{R}(\mathbf{b}) = \bar{\Delta\mathbf{R}} \cdot \text{Exp}\!\left(\mathbf{J}^R_g \cdot \delta\mathbf{b}^g\right)$$

$$\Delta\mathbf{v}(\mathbf{b}) = \bar{\Delta\mathbf{v}} + \mathbf{J}^v_g \cdot \delta\mathbf{b}^g + \mathbf{J}^v_a \cdot \delta\mathbf{b}^a$$

$$\Delta\mathbf{p}(\mathbf{b}) = \bar{\Delta\mathbf{p}} + \mathbf{J}^p_g \cdot \delta\mathbf{b}^g + \mathbf{J}^p_a \cdot \delta\mathbf{b}^a$$

### 4.2 Residual（`G2oTypes.cc:514`）

**旋轉殘差**（與 EdgeInertialGS 相同）：

$$\mathbf{r}_{\Delta R} = \text{Log}\!\left(\Delta\mathbf{R}(\mathbf{b})^T \cdot \mathbf{R}_{wb_1}^T \cdot \mathbf{R}_{wb_2}\right)$$

**速度殘差**（無 scale $s$）：

$$\mathbf{r}_{\Delta v} = \mathbf{R}_{wb_1}^T \left(\mathbf{v}_2 - \mathbf{v}_1 - \mathbf{g}\cdot\Delta t\right) - \Delta\mathbf{v}(\mathbf{b})$$

**位置殘差**（無 scale $s$）：

$$\mathbf{r}_{\Delta p} = \mathbf{R}_{wb_1}^T \left(\mathbf{t}_{wb_2} - \mathbf{t}_{wb_1} - \mathbf{v}_1\Delta t - \frac{1}{2}\mathbf{g}\Delta t^2\right) - \Delta\mathbf{p}(\mathbf{b})$$

和 `EdgeInertialGS` 的差異：速度和位置殘差中不乘 scale $s$，因為 scale 已套用到整張地圖。

### 4.3 Information Matrix

$$\Omega_i = \mathbf{C}_{[0:9,\ 0:9]}^{-1}$$

與 `EdgeInertialGS` 相同，取預積分 covariance 矩陣的前 9×9 塊反矩陣。

### 4.4 Robust Kernel

```
RobustKernelHuber，delta = sqrt(16.92)
```

`EdgeInertialGS` 沒有 robust kernel，`EdgeInertial` 有。

---

## 5. Bias Random Walk（`EdgeGyroRW` + `EdgeAccRW`）

**僅 `bInit = false` 時使用。**

IMU 的 bias 不是完全固定的常數，會隨時間緩慢漂移（random walk 模型）。這條邊約束相鄰 KF 的 bias 差異不要太大。

### Residual（`G2oTypes.h:645`、`G2oTypes.h:681`）

$$\mathbf{r}_{\text{gyroRW}} = \mathbf{b}^g_{i} - \mathbf{b}^g_{i-1}$$

$$\mathbf{r}_{\text{accRW}} = \mathbf{b}^a_{i} - \mathbf{b}^a_{i-1}$$

bias 在相鄰 KF 間應盡量不變，差值越小越好。

### Information Matrix

$$\Omega_{\text{gyroRW}} = \mathbf{C}_{[9:12,\ 9:12]}^{-1}$$

$$\Omega_{\text{accRW}} = \mathbf{C}_{[12:15,\ 12:15]}^{-1}$$

取預積分 covariance 矩陣中 gyro bias（9:12）和 acc bias（12:15）對應的塊反矩陣。

### Jacobian

$$\frac{\partial \mathbf{r}}{\partial \mathbf{b}^g_{i-1}} = -I_3, \quad \frac{\partial \mathbf{r}}{\partial \mathbf{b}^g_i} = +I_3$$

---

## 6. 視覺 Reprojection Error

### Monocular（`EdgeMono`，`G2oTypes.h:353`）

$$\mathbf{r}_{\text{mono}} = \mathbf{u}_{obs} - \Pi\!\left(\mathbf{T}_{cw} \cdot \mathbf{X}_w\right)$$

- $\mathbf{u}_{obs} \in \mathbb{R}^2$：觀測到的像素座標
- $\Pi$：投影函式（針孔或魚眼，依 camera model）
- $\mathbf{T}_{cw}$：world-to-camera 變換，由 `VertexPose` 提供
- information：$\sigma^{-2}_{\text{level}} \cdot I_2$（依特徵點金字塔層數縮放）
- Huber kernel：$\delta = \sqrt{5.991}$（$\chi^2$，2 自由度，95%）

### Stereo（`EdgeStereo`，`G2oTypes.h:435`）

$$\mathbf{r}_{\text{stereo}} = \begin{bmatrix} u \\ v \\ u_r \end{bmatrix}_{obs} - \Pi_{\text{stereo}}\!\left(\mathbf{T}_{cw} \cdot \mathbf{X}_w\right)$$

- 多一維：右相機的 $u_r$（水平視差）
- information：$\sigma^{-2}_{\text{level}} \cdot I_3$
- Huber kernel：$\delta = \sqrt{7.815}$（$\chi^2$，3 自由度，95%）

---

## 7. Bias Prior（bInit = true 時）

與 `InertialOptimization` 完全相同：

$$\mathbf{r}_{\text{prior},g} = \mathbf{0} - \mathbf{b}^g, \quad \Omega = \text{priorG} \cdot I_3$$

$$\mathbf{r}_{\text{prior},a} = \mathbf{0} - \mathbf{b}^a, \quad \Omega = \text{priorA} \cdot I_3$$

---

## 8. 優化流程

```
1. 建立頂點：
   VertexPose × N（可優化）
   VertexVelocity × N
   VertexGyroBias × N（bInit=false）或 × 1（bInit=true）
   VertexAccBias  × N（bInit=false）或 × 1（bInit=true）
   VertexSBAPointXYZ × M（marginalized）

2. 建立邊：
   EdgeInertial × N          → IMU preintegration（Huber kernel）
   EdgeGyroRW × N            → bias 連續性（bInit=false 才有）
   EdgeAccRW × N             → bias 連續性（bInit=false 才有）
   EdgeMono / EdgeStereo × K → 視覺 reprojection（Huber kernel）
   EdgePriorGyro             → bias prior（bInit=true 才有）
   EdgePriorAcc              → bias prior（bInit=true 才有）

3. Levenberg-Marquardt 迭代 100 次（its 參數傳入）
   λ 初始值 = 1e-5（比 InertialOptimization 的 1e3 小很多）

4. MapPoint 被 marginalized，Schur complement 消去後
   實際求解變數只剩 KF 的位姿、速度、bias
```

---

## 9. 和 InertialOptimization 的關鍵差異

| | `InertialOptimization` | `FullInertialBA` |
|-|------------------------|-----------------|
| KF 位姿 | 固定 | **可優化** |
| scale $s$ | 優化變數 | 不存在 |
| 重力 $\mathbf{R}_{wg}$ | 優化變數 | 不存在（$\mathbf{g}$ 為固定常數）|
| bias | 共用一組 | 每 KF 一組（bInit=false）|
| IMU edge | `EdgeInertialGS`（含 $s$, $\mathbf{R}_{wg}$）| `EdgeInertial`（不含）|
| 速度/位置殘差 | 乘 scale $s$ | **不乘** $s$ |
| bias 連續性 | 無 | `EdgeGyroRW` + `EdgeAccRW` |
| 視覺項 | 無 | `EdgeMono` + `EdgeStereo` |
| IMU edge robust kernel | 無 | Huber（$\delta = \sqrt{16.92}$）|
| LM 初始 $\lambda$ | 1e3（有 prior 時）| 1e-5 |
