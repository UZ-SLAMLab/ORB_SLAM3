# InertialOptimization 完整推導（k=2）

從頭到尾推導一次 ORB-SLAM3 IMU 初始化（InertialOptimization），設定 k=2（2 個 KF：KF₀ 和 KF₁，1 條 EdgeInertialGS）。每個數學步驟對應到具體程式碼行號。

---

## Step 1：初始值計算

**對應程式：** `LocalMapping.cc:1322-1356`

進入 `InitializeIMU` 時，已知量（從視覺 SLAM）：
- $\mathbf{R}_{wb_0}, \mathbf{t}_{wb_0}$：KF₀ 位姿（fixed）
- $\mathbf{R}_{wb_1}, \mathbf{t}_{wb_1}$：KF₁ 位姿（fixed）
- KF₀→KF₁ 的預積分量：$\bar{\Delta\mathbf{R}}, \bar{\Delta\mathbf{v}}, \bar{\Delta\mathbf{p}}$
- 對 bias 的 Jacobian：$\mathbf{J}^R_g, \mathbf{J}^v_g, \mathbf{J}^v_a, \mathbf{J}^p_g, \mathbf{J}^p_a$
- Covariance $\mathbf{C} \in \mathbb{R}^{15\times15}$
- 時間差 $\Delta t$

### 1.1 粗估速度（line 1336）

對 KF₁（唯一有 mPrevKF 的 KF）：

$$\mathbf{v}_0 = \mathbf{v}_1 = \frac{\mathbf{t}_{wb_1} - \mathbf{t}_{wb_0}}{\Delta t}$$

兩個 KF 的速度被設成同一個粗估值。

### 1.2 累積重力方向（line 1335）

$$\mathbf{dirG} = -\mathbf{R}_{wb_0} \cdot \bar{\Delta\mathbf{v}}_{\text{updated}}$$

`GetUpdatedDeltaVelocity()` 回傳的是用目前 bias（這裡 b=0）修正後的 $\bar{\Delta\mathbf{v}}$。$\bar{\Delta\mathbf{v}}$ 是 IMU body frame 下從 KF₀ 到 KF₁ 的速度變化（包含重力影響）。

### 1.3 計算 Rwg（line 1341-1349）

歸一化：

$$\hat{\mathbf{dirG}} = \frac{\mathbf{dirG}}{\|\mathbf{dirG}\|}$$

定義目標方向：

$$\mathbf{g}_I = [0, 0, -1]^T$$

算旋轉軸（外積）：

$$\mathbf{v} = \mathbf{g}_I \times \hat{\mathbf{dirG}}$$

算旋轉角：

$$\cos\theta = \mathbf{g}_I \cdot \hat{\mathbf{dirG}}, \quad \theta = \arccos(\cos\theta)$$

組成軸角向量：

$$\mathbf{v}_{zg} = \mathbf{v} \cdot \frac{\theta}{\|\mathbf{v}\|}$$

轉成旋轉矩陣：

$$\mathbf{R}_{wg} = \text{Exp}(\mathbf{v}_{zg})$$

### 1.4 其他初始值

```
s = 1.0                  (line 1357)
bg = 0, ba = 0           (b 初始為零)
```

---

## Step 2：InertialOptimization 建立 Graph

**對應程式：** `Optimizer.cc:3042-3170`

### 2.1 LM 求解器設定（line 3057-3061）

```cpp
g2o::OptimizationAlgorithmLevenberg* solver = ...;
if (priorG != 0.f)
    solver->setUserLambdaInit(1e3);  // 初始 λ = 1000
```

priorG=0 時用 g2o 預設 $\lambda$（通常根據 H 的對角線元素自動算）。

### 2.2 頂點清單

k=2 時：

| 頂點 | 維度 | 初始值 | fixed? |
|------|------|--------|--------|
| `VertexPose` VP₀ | 6 | $\mathbf{R}_{wb_0}, \mathbf{t}_{wb_0}$ | **true** |
| `VertexPose` VP₁ | 6 | $\mathbf{R}_{wb_1}, \mathbf{t}_{wb_1}$ | **true** |
| `VertexVelocity` VV₀ | 3 | $\mathbf{v}_0$ | false |
| `VertexVelocity` VV₁ | 3 | $\mathbf{v}_1$ | false |
| `VertexGyroBias` VG | 3 | $[0,0,0]^T$ | false |
| `VertexAccBias` VA | 3 | $[0,0,0]^T$ | false |
| `VertexGDir` VGDir | 2 | $\mathbf{R}_{wg}$ | false |
| `VertexScale` VS | 1 | 1.0 | mono=false / stereo=true |

優化變數共 **3+3+3+3+2+1 = 15 維**（mono），位姿 12 維被 fix 不算。

### 2.3 邊清單

**EdgePriorGyro**（line 3119-3123）：
```
連接：VG
information = priorG · I₃
```

**EdgePriorAcc**（line 3113-3118）：
```
連接：VA
information = priorA · I₃
```

**EdgeInertialGS**（line 3151-3160）：
```
連接：VP₀, VV₀, VG, VA, VP₁, VV₁, VGDir, VS
information = (C[0:9, 0:9])^{-1}（並做特徵值正則化，line 600-609）
```

---

## Step 3：computeError()

**對應程式：** `G2oTypes.cc:617-640`

### 3.1 Bias 修正（ImuTypes.cc:283-308）

設 $\bar{\mathbf{b}}$ 是預積分時的 bias 線性化點（這裡是 0），目前估計 $\mathbf{b} = (\mathbf{b}^a, \mathbf{b}^g)$，差值：

$$\delta\mathbf{b}^g = \mathbf{b}^g - \bar{\mathbf{b}}^g, \quad \delta\mathbf{b}^a = \mathbf{b}^a - \bar{\mathbf{b}}^a$$

第一次迭代 $\mathbf{b} = 0$，所以 $\delta\mathbf{b}^g = \delta\mathbf{b}^a = 0$，但之後每次迭代會變。

修正後的預積分量：

$$\Delta\mathbf{R}(\mathbf{b}) = \bar{\Delta\mathbf{R}} \cdot \text{Exp}(\mathbf{J}^R_g \cdot \delta\mathbf{b}^g)$$

$$\Delta\mathbf{v}(\mathbf{b}) = \bar{\Delta\mathbf{v}} + \mathbf{J}^v_g \cdot \delta\mathbf{b}^g + \mathbf{J}^v_a \cdot \delta\mathbf{b}^a$$

$$\Delta\mathbf{p}(\mathbf{b}) = \bar{\Delta\mathbf{p}} + \mathbf{J}^p_g \cdot \delta\mathbf{b}^g + \mathbf{J}^p_a \cdot \delta\mathbf{b}^a$$

### 3.2 重力向量（line 631）

$$\mathbf{g} = \mathbf{R}_{wg} \cdot \mathbf{g}_I, \quad \mathbf{g}_I = [0, 0, -9.81]^T$$

### 3.3 旋轉殘差（line 636）

$$\mathbf{r}_{\Delta R} = \text{Log}\!\left(\Delta\mathbf{R}(\mathbf{b})^T \cdot \mathbf{R}_{wb_0}^T \cdot \mathbf{R}_{wb_1}\right)$$

定義中間量：

$$\mathbf{e}_R = \Delta\mathbf{R}(\mathbf{b})^T \cdot \mathbf{R}_{wb_0}^T \cdot \mathbf{R}_{wb_1}$$

$$\mathbf{r}_{\Delta R} = \text{Log}(\mathbf{e}_R) \in \mathbb{R}^3$$

### 3.4 速度殘差（line 637）

$$\mathbf{r}_{\Delta v} = \mathbf{R}_{wb_0}^T \left(s(\mathbf{v}_1 - \mathbf{v}_0) - \mathbf{g}\Delta t\right) - \Delta\mathbf{v}(\mathbf{b}) \in \mathbb{R}^3$$

### 3.5 位置殘差（line 638）

$$\mathbf{r}_{\Delta p} = \mathbf{R}_{wb_0}^T \left(s(\mathbf{t}_{wb_1} - \mathbf{t}_{wb_0} - \mathbf{v}_0 \Delta t) - \frac{1}{2}\mathbf{g}\Delta t^2\right) - \Delta\mathbf{p}(\mathbf{b}) \in \mathbb{R}^3$$

### 3.6 合併成 9 維殘差

$$\mathbf{r}_{\mathcal{I}} = \begin{bmatrix} \mathbf{r}_{\Delta R} \\ \mathbf{r}_{\Delta v} \\ \mathbf{r}_{\Delta p} \end{bmatrix} \in \mathbb{R}^9$$

### 3.7 Prior 殘差（G2oTypes.h:779, 805）

$$\mathbf{r}_{pg} = \mathbf{0} - \mathbf{b}^g \in \mathbb{R}^3$$

$$\mathbf{r}_{pa} = \mathbf{0} - \mathbf{b}^a \in \mathbb{R}^3$$

### 3.8 Total cost

$$C = \mathbf{r}_{\mathcal{I}}^T \mathbf{\Omega}_I \mathbf{r}_{\mathcal{I}} + \mathbf{r}_{pg}^T (\text{priorG} \cdot I_3) \mathbf{r}_{pg} + \mathbf{r}_{pa}^T (\text{priorA} \cdot I_3) \mathbf{r}_{pa}$$

其中 $\mathbf{\Omega}_I = (\mathbf{C}_{[0:9,0:9]})^{-1}$。

---

## Step 4：linearizeOplus()

**對應程式：** `G2oTypes.cc:642-738`（EdgeInertialGS）、`G2oTypes.cc:762-775`（EdgePriorGyro / EdgePriorAcc）

優化變數順序：$\mathbf{x} = [\mathbf{v}_0, \mathbf{v}_1, \mathbf{b}^g, \mathbf{b}^a, \mathbf{R}_{wg}, s]$，共 15 維（mono；stereo 時 $s$ fixed，少 1 維變 14 維）。

每條邊各自負責計算自己對所連接頂點的 Jacobian，存進 `_jacobianOplus[i]`。g2o 在組裝 H 時自動跳過 fixed 頂點對應的 block。

### 4.1 EdgeInertialGS Jacobian（9×15，連接 8 頂點）

中間量：
- $\mathbf{R}_{bw_0} = \mathbf{R}_{wb_0}^T$
- $\mathbf{e}_R = \Delta\mathbf{R}(\mathbf{b})^T \mathbf{R}_{bw_0} \mathbf{R}_{wb_1}$（這是 $\mathbf{r}_{\Delta R}$ 取 Log 之前的旋轉矩陣）
- $\mathbf{J}_r^{-1} = \text{InverseRightJacobianSO3}(\text{Log}(\mathbf{e}_R))$
- $\mathbf{G}_m = \begin{bmatrix} 0 & -9.81 \\ 9.81 & 0 \\ 0 & 0 \end{bmatrix}$
- $\frac{\partial\mathbf{g}}{\partial\theta} = \mathbf{R}_{wg} \cdot \mathbf{G}_m$
- $\delta\mathbf{b}^g$ 的當前值（從 GetDeltaBias 拿）

**InverseRightJacobianSO3 是什麼？**

SO(3) 的右 Jacobian $\mathbf{J}_r(\boldsymbol{\phi})$ 連接「旋轉向量空間」和「旋轉矩陣流形」之間的微分關係：

$$\text{Exp}(\boldsymbol{\phi} + \delta\boldsymbol{\phi}) \approx \text{Exp}(\boldsymbol{\phi}) \cdot \text{Exp}(\mathbf{J}_r(\boldsymbol{\phi}) \cdot \delta\boldsymbol{\phi})$$

逆右 Jacobian $\mathbf{J}_r^{-1}(\boldsymbol{\phi})$ 是反方向：

$$\text{Log}(\text{Exp}(\boldsymbol{\phi}) \cdot \text{Exp}(\delta\boldsymbol{\phi})) \approx \boldsymbol{\phi} + \mathbf{J}_r^{-1}(\boldsymbol{\phi}) \cdot \delta\boldsymbol{\phi}$$

**為什麼這裡需要 $\mathbf{J}_r^{-1}$？**

旋轉殘差 $\mathbf{r}_{\Delta R} = \text{Log}(\mathbf{e}_R)$ 經過 Log 映射，要對 $\mathbf{R}_{wb_1}$、$\mathbf{R}_{wb_0}$、$\mathbf{b}^g$ 等變數做偏微分時，需要 $\mathbf{J}_r^{-1}$ 把流形上的變化「拉回」到向量空間：

$$\frac{\partial \text{Log}(\mathbf{e}_R)}{\partial \mathbf{R}_{wb_1}} \propto \mathbf{J}_r^{-1}(\text{Log}(\mathbf{e}_R))$$

所以你會在 $\partial \mathbf{r}_{\Delta R} / \partial \cdot$ 的所有公式裡看到 $\mathbf{J}_r^{-1}$ 出現。

**閉式公式**（G2oTypes.cc:821-832）：

設 $\boldsymbol{\phi} = (x, y, z)$，$d = \|\boldsymbol{\phi}\|$，$\mathbf{W} = [\boldsymbol{\phi}]_\times$（hat operator）：

$$\mathbf{J}_r^{-1}(\boldsymbol{\phi}) = \mathbf{I}_3 + \frac{1}{2}\mathbf{W} + \left(\frac{1}{d^2} - \frac{1+\cos d}{2d \sin d}\right)\mathbf{W}^2$$

當 $d < 10^{-5}$（小角度）退化為 $\mathbf{I}_3$，避免數值問題。

對應的 `RightJacobianSO3`：

$$\mathbf{J}_r(\boldsymbol{\phi}) = \mathbf{I}_3 - \frac{1-\cos d}{d^2}\mathbf{W} + \frac{d - \sin d}{d^3}\mathbf{W}^2$$

**為什麼 $\partial \mathbf{r}_{\Delta R} / \partial \mathbf{b}^g$ 裡同時出現 $\mathbf{J}_r^{-1}$ 和 $\mathbf{J}_r$？**

$$\frac{\partial \mathbf{r}_{\Delta R}}{\partial \mathbf{b}^g} = -\mathbf{J}_r^{-1}(\mathbf{r}_{\Delta R}) \cdot \mathbf{e}_R^T \cdot \mathbf{J}_r(\mathbf{J}^R_g \delta\mathbf{b}^g) \cdot \mathbf{J}^R_g$$

- $\mathbf{J}_r^{-1}(\mathbf{r}_{\Delta R})$：把 Log 後的向量微分**拉回**到流形
- $\mathbf{J}_r(\mathbf{J}^R_g \delta\mathbf{b}^g)$：bias 變化引起 $\Delta\mathbf{R}(\mathbf{b}) = \bar{\Delta\mathbf{R}} \cdot \text{Exp}(\mathbf{J}^R_g \delta\mathbf{b}^g)$，這個 Exp 的右 Jacobian 把 bias 空間的小變化**推到**流形上的旋轉變化
- 兩個方向的「轉換」串接，加上鏈式法則的中間項 $\mathbf{e}_R^T$ 和 $\mathbf{J}^R_g$，得到完整的偏微分

**對 $\mathbf{v}_0$（3 欄，line 690-693）**：

$$\frac{\partial \mathbf{r}_{\Delta R}}{\partial \mathbf{v}_0} = \mathbf{0}_{3\times 3}$$

$$\frac{\partial \mathbf{r}_{\Delta v}}{\partial \mathbf{v}_0} = -s \cdot \mathbf{R}_{bw_0}$$

$$\frac{\partial \mathbf{r}_{\Delta p}}{\partial \mathbf{v}_0} = -s \cdot \mathbf{R}_{bw_0} \cdot \Delta t$$

**對 $\mathbf{v}_1$（3 欄，line 717-719）**：

$$\frac{\partial \mathbf{r}_{\Delta R}}{\partial \mathbf{v}_1} = \mathbf{0}_{3\times 3}$$

$$\frac{\partial \mathbf{r}_{\Delta v}}{\partial \mathbf{v}_1} = s \cdot \mathbf{R}_{bw_0}$$

$$\frac{\partial \mathbf{r}_{\Delta p}}{\partial \mathbf{v}_1} = \mathbf{0}_{3\times 3}$$

**對 $\mathbf{b}^g$（3 欄，line 695-699）**：

$$\frac{\partial \mathbf{r}_{\Delta R}}{\partial \mathbf{b}^g} = -\mathbf{J}_r^{-1} \cdot \mathbf{e}_R^T \cdot \mathbf{J}_r(\mathbf{J}^R_g \cdot \delta\mathbf{b}^g) \cdot \mathbf{J}^R_g$$

$$\frac{\partial \mathbf{r}_{\Delta v}}{\partial \mathbf{b}^g} = -\mathbf{J}^v_g$$

$$\frac{\partial \mathbf{r}_{\Delta p}}{\partial \mathbf{b}^g} = -\mathbf{J}^p_g$$

**對 $\mathbf{b}^a$（3 欄，line 701-704）**：

$$\frac{\partial \mathbf{r}_{\Delta R}}{\partial \mathbf{b}^a} = \mathbf{0}_{3\times 3}$$

$$\frac{\partial \mathbf{r}_{\Delta v}}{\partial \mathbf{b}^a} = -\mathbf{J}^v_a$$

$$\frac{\partial \mathbf{r}_{\Delta p}}{\partial \mathbf{b}^a} = -\mathbf{J}^p_a$$

**對 $\mathbf{R}_{wg}$（2 欄，line 721-723）**：

$$\frac{\partial \mathbf{r}_{\Delta R}}{\partial (\alpha, \beta)} = \mathbf{0}_{3\times 2}$$

$$\frac{\partial \mathbf{r}_{\Delta v}}{\partial (\alpha, \beta)} = -\mathbf{R}_{bw_0} \cdot \mathbf{R}_{wg} \cdot \mathbf{G}_m \cdot \Delta t$$

$$\frac{\partial \mathbf{r}_{\Delta p}}{\partial (\alpha, \beta)} = -\frac{1}{2} \mathbf{R}_{bw_0} \cdot \mathbf{R}_{wg} \cdot \mathbf{G}_m \cdot \Delta t^2$$

**對 $s$（1 欄，line 726-727）**：

$$\frac{\partial \mathbf{r}_{\Delta R}}{\partial s} = \mathbf{0}_{3\times 1}$$

$$\frac{\partial \mathbf{r}_{\Delta v}}{\partial s} = \mathbf{R}_{bw_0} (\mathbf{v}_1 - \mathbf{v}_0)$$

$$\frac{\partial \mathbf{r}_{\Delta p}}{\partial s} = \mathbf{R}_{bw_0} (\mathbf{t}_{wb_1} - \mathbf{t}_{wb_0} - \mathbf{v}_0 \Delta t)$$

**對 $\mathbf{T}_0, \mathbf{T}_1$（pose 頂點，line 670-687、712-715）**：

程式碼**有計算** Pose Jacobian，但這兩個頂點 `setFixed(true)`，g2o 在組裝 H 時自動忽略對應 block，所以這些 Jacobian 算了不會影響 δx：

```cpp
// _jacobianOplus[0] (對 T0)：rotation 和 translation 都有非零 block
_jacobianOplus[0].block<3,3>(0,0) = -invJr*Rwb2.transpose()*Rwb1;  // 用不到
_jacobianOplus[0].block<3,3>(6,3) = DiagonalMatrix(-s,-s,-s);       // 用不到
// _jacobianOplus[4] (對 T1)：同樣計算了但用不到
```

可以視為「程式碼為了通用性實作完整版，InertialOptimization 只用其中部分」。

### 4.2 EdgePriorGyro / EdgePriorAcc Jacobian（3×3 each，line 762-775）

兩條 unary edge，殘差 $\mathbf{r} = \mathbf{0} - \mathbf{b}$：

$$\frac{\partial \mathbf{r}_{pg}}{\partial \mathbf{b}^g} = +\mathbf{I}_3, \quad \frac{\partial \mathbf{r}_{pa}}{\partial \mathbf{b}^a} = +\mathbf{I}_3$$

```cpp
void EdgePriorGyro::linearizeOplus()
{
    _jacobianOplusXi.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
}
```

理論上應為 $-\mathbf{I}_3$（因為 $\mathbf{r} = -\mathbf{b}$），程式碼寫 $+\mathbf{I}_3$ 是符號錯誤，但因 $J^T \Omega J = \Omega$ 與正負無關，Hessian 不受影響，gradient 方向會反但 prior 是弱約束所以不影響收斂。

### 4.3 完整 J 矩陣（15×15，block 形式）

$$\mathbf{J} = \begin{bmatrix}
\mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & -\mathbf{J}_r^{-1}\mathbf{e}_R^T\mathbf{J}_r(\mathbf{J}^R_g\delta\mathbf{b}^g)\mathbf{J}^R_g & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times2} & \mathbf{0}_{3\times1} \\
-s\mathbf{R}_{bw_0} & s\mathbf{R}_{bw_0} & -\mathbf{J}^v_g & -\mathbf{J}^v_a & -\mathbf{R}_{bw_0}\mathbf{R}_{wg}\mathbf{G}_m\Delta t & \mathbf{R}_{bw_0}(\mathbf{v}_1-\mathbf{v}_0) \\
-s\mathbf{R}_{bw_0}\Delta t & \mathbf{0}_{3\times3} & -\mathbf{J}^p_g & -\mathbf{J}^p_a & -\tfrac{1}{2}\mathbf{R}_{bw_0}\mathbf{R}_{wg}\mathbf{G}_m\Delta t^2 & \mathbf{R}_{bw_0}(\mathbf{t}_1-\mathbf{t}_0-\mathbf{v}_0\Delta t) \\
\mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & -\mathbf{I}_3 & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times2} & \mathbf{0}_{3\times1} \\
\mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & -\mathbf{I}_3 & \mathbf{0}_{3\times2} & \mathbf{0}_{3\times1}
\end{bmatrix}$$

欄位順序：$[\mathbf{v}_0, \mathbf{v}_1, \mathbf{b}^g, \mathbf{b}^a, \mathbf{R}_{wg}, s]$，列順序：$[\mathbf{r}_{\Delta R}, \mathbf{r}_{\Delta v}, \mathbf{r}_{\Delta p}, \mathbf{r}_{pg}, \mathbf{r}_{pa}]$。

注意：prior 的 Jacobian 程式碼寫 `+I₃`（G2oTypes.cc:762-775），但 residual 是 `0 - b`，理論應為 `-I₃`（這裡按理論值寫）。這個符號不影響 Hessian（$J^T \Omega J = \Omega$ 無論正負），實務上不影響收斂。

### 4.4 J 拆成各邊獨立的 Jacobian

g2o 內部不組成完整 J，而是每條邊各自存 Jacobian：

- `EdgeInertialGS::_jacobianOplus[1..7]`：上面 9 列（rows 0-8），每個對應一個頂點
- `EdgePriorGyro::_jacobianOplusXi`：3×3，對 bg
- `EdgePriorAcc::_jacobianOplusXi`：3×3，對 ba

組裝 H 時 g2o 把它們疊加進對應的 block。

---

## Step 5：組裝 H 和 b

g2o 不是直接組大 J，而是用稀疏方式累加 H 的 block。為了清楚起見，這裡用完整 J 表達。

### 5.1 完整 information matrix

$$\mathbf{\Omega}_{\text{total}} = \text{block-diag}(\mathbf{\Omega}_I, \text{priorG} \cdot I_3, \text{priorA} \cdot I_3) \in \mathbb{R}^{15\times 15}$$

### 5.2 Hessian 近似

$$\mathbf{H} = \mathbf{J}^T \mathbf{\Omega}_{\text{total}} \mathbf{J} \in \mathbb{R}^{15\times 15}$$

拆解成各邊貢獻：

$$\mathbf{H} = \mathbf{J}_{GS}^T \mathbf{\Omega}_I \mathbf{J}_{GS} + \mathbf{J}_{pg}^T (\text{priorG} \cdot I_3) \mathbf{J}_{pg} + \mathbf{J}_{pa}^T (\text{priorA} \cdot I_3) \mathbf{J}_{pa}$$

其中：
- $\mathbf{J}_{GS}$：EdgeInertialGS 的 9×15 Jacobian
- $\mathbf{J}_{pg}$：EdgePriorGyro 的 3×15 Jacobian（只有 bg 那 3 欄非零）
- $\mathbf{J}_{pa}$：EdgePriorAcc 的 3×15 Jacobian（只有 ba 那 3 欄非零）

### 5.3 Gradient

$$\mathbf{b} = \mathbf{J}^T \mathbf{\Omega}_{\text{total}} \mathbf{r} \in \mathbb{R}^{15}$$

$$\mathbf{b} = \mathbf{J}_{GS}^T \mathbf{\Omega}_I \mathbf{r}_{\mathcal{I}} + \mathbf{J}_{pg}^T (\text{priorG} \cdot I_3) \mathbf{r}_{pg} + \mathbf{J}_{pa}^T (\text{priorA} \cdot I_3) \mathbf{r}_{pa}$$

---

## Step 6：解 (H + λI)δx = -b

### 6.1 λ 的初始值

```cpp
if (priorG != 0.f)
    solver->setUserLambdaInit(1e3);
```

- 初始化和 VIBA 1：$\lambda_0 = 10^3$
- VIBA 2（priorG=0）：g2o 預設，$\lambda_0 = \tau \cdot \max(\text{diag}(H))$，$\tau$ 預設 $10^{-5}$

### 6.2 解線性方程

$$(\mathbf{H} + \lambda \mathbf{I})\, \delta\mathbf{x} = -\mathbf{b}$$

g2o 用 `LinearSolverEigen<BlockSolverX::PoseMatrixType>`，對稀疏對稱矩陣做 Cholesky 分解求解。

得到（與 Step 4 變數順序對應，共 15 維）：

$$\delta\mathbf{x} = [\underbrace{\delta\mathbf{v}_0}_{3},\ \underbrace{\delta\mathbf{v}_1}_{3},\ \underbrace{\delta\mathbf{b}^g}_{3},\ \underbrace{\delta\mathbf{b}^a}_{3},\ \underbrace{\delta\mathbf{R}_{wg}}_{2},\ \underbrace{\delta s}_{1}] \in \mathbb{R}^{15}$$

其中 $\delta\mathbf{R}_{wg} = (\delta\alpha, \delta\beta)$ 是 2-DOF 流形更新（z 軸鎖死），$\delta s$ 是純量（stereo 時 fixed，δx 變 14 維）。

---

## Step 7：oplusImpl()

**對應程式：** `G2oTypes.h`

### 7.1 VertexVelocity（line 205-208）

$$\mathbf{v}_0^{\text{new}} = \mathbf{v}_0^{\text{old}} + \delta\mathbf{v}_0$$

$$\mathbf{v}_1^{\text{new}} = \mathbf{v}_1^{\text{old}} + \delta\mathbf{v}_1$$

### 7.2 VertexGyroBias（line 226-229）

$$\mathbf{b}^{g,\text{new}} = \mathbf{b}^{g,\text{old}} + \delta\mathbf{b}^g$$

### 7.3 VertexAccBias（line 248-251）

$$\mathbf{b}^{a,\text{new}} = \mathbf{b}^{a,\text{old}} + \delta\mathbf{b}^a$$

### 7.4 VertexGDir（line 264, 289-291）

$$\mathbf{R}_{wg}^{\text{new}} = \mathbf{R}_{wg}^{\text{old}} \cdot \text{Exp}(\delta\alpha, \delta\beta, 0)$$

### 7.5 VertexScale（line 314-316）

$$s^{\text{new}} = s^{\text{old}} \cdot \exp(\delta s)$$

---

## Step 8：驗證 cost 並調整 λ

g2o 的 LM 內部判斷：

### 8.1 算新 cost

用更新後的 $\mathbf{x}^{\text{new}}$ 重新算 $\mathbf{r}^{\text{new}}$ 和 $C^{\text{new}}$。

### 8.2 增益比 $\rho$

$$\rho = \frac{C^{\text{old}} - C^{\text{new}}}{\Delta L(\delta\mathbf{x})}$$

其中 $\Delta L(\delta\mathbf{x}) = \delta\mathbf{x}^T (\lambda \delta\mathbf{x} - \mathbf{b})$ 是線性模型的預期下降量。

### 8.3 λ 調整規則

- **$\rho > 0$**（cost 下降）：接受更新，$\lambda \leftarrow \lambda \cdot \max\!\left(\frac{1}{3}, 1 - (2\rho - 1)^3\right)$（縮小）
- **$\rho \leq 0$**（cost 上升）：拒絕更新，$\lambda \leftarrow \lambda \cdot 2$（放大），重新解

g2o 預設用這套 Marquardt 風格的調整。

---

## Step 9：重複到收斂

回到 Step 3，用新的 $\mathbf{x}$ 重新算 residual、Jacobian、H、b、$\delta\mathbf{x}$、更新。最多 200 次（`int its = 200`），實際會提早因下列任一條件停止：

- $\|\delta\mathbf{x}\|$ 太小
- cost 變化太小
- gradient 太小

---

## Step 10：讀回結果

**對應程式：** `Optimizer.cc:3193-3220`

```cpp
scale = VS->estimate();              // s
Rwg   = VGDir->estimate().Rwg;       // Rwg
bg    = VG->estimate();              // bg
ba    = VA->estimate();              // ba

for each KF:
    Vw = VV->estimate();
    pKFi->SetVelocity(Vw);            // v0, v1

    if ‖bg_old - bg_new‖ > 0.01:
        pKFi->SetNewBias(b);
        pKFi->mpImuPreintegrated->Reintegrate();  // 重新積分
    else:
        pKFi->SetNewBias(b);
```

bias 變化太大時呼叫 `Reintegrate()`：用新 bias 重新對所有原始 IMU 資料做積分，更新 $\bar{\Delta\mathbf{R}}, \bar{\Delta\mathbf{v}}, \bar{\Delta\mathbf{p}}$（不再依賴 Taylor 展開近似）。

---

## Step 11：ApplyScaledRotation

**對應程式：** `LocalMapping.cc:1379-1380`、`Map.cc:252-282`

### 11.1 建立變換

```cpp
Sophus::SE3f Twg(mRwg.cast<float>().transpose(), Eigen::Vector3f::Zero());
mpAtlas->GetCurrentMap()->ApplyScaledRotation(Twg, mScale, true);
```

定義：

$$\mathbf{T}_{yw} = (\mathbf{R}_{wg}^T,\ \mathbf{0}), \quad \mathbf{R}_{yw} = \mathbf{R}_{wg}^T, \quad \mathbf{t}_{yw} = \mathbf{0}$$

### 11.2 對每個 KF 套用（Map.cc:262-273）

設 KF 原本的 pose 是 $\mathbf{T}_{cw}$，相機到世界 $\mathbf{T}_{wc} = \mathbf{T}_{cw}^{-1}$：

**先 scale 平移**：

$$\mathbf{t}_{wc}^{\text{scaled}} = s \cdot \mathbf{t}_{wc}$$

**再做變換**：

$$\mathbf{T}_{yc} = \mathbf{T}_{yw} \cdot \mathbf{T}_{wc}^{\text{scaled}}$$

$$\mathbf{T}_{cy}^{\text{new}} = \mathbf{T}_{yc}^{-1}$$

`pKF->SetPose(Tcy)` 寫回。

**速度更新**（`bScaledVel=true`）：

$$\mathbf{v}^{\text{new}} = \mathbf{R}_{yw} \cdot \mathbf{v}^{\text{opt}} \cdot s$$

### 11.3 對每個 MapPoint 套用（Map.cc:278）

$$\mathbf{p}_{MP}^{\text{new}} = s \cdot \mathbf{R}_{yw} \cdot \mathbf{p}_{MP}^{\text{old}} + \mathbf{t}_{yw}$$

由於 $\mathbf{t}_{yw} = \mathbf{0}$：

$$\mathbf{p}_{MP}^{\text{new}} = s \cdot \mathbf{R}_{yw} \cdot \mathbf{p}_{MP}^{\text{old}}$$

---

## 整體流程總結

```
粗估初始值（dirG, Rwg, v）
    ↓
建 graph（8 頂點 + 3 邊）
    ↓
┌──────────────────────────────────┐
│  LM 迭代（最多 200 次）           │
│  ├─ computeError → 9+3+3 維 r    │
│  ├─ linearizeOplus → J 各 block  │
│  ├─ H = J^T Ω J, b = J^T Ω r     │
│  ├─ 解 (H+λI)δx = -b              │
│  ├─ oplusImpl 套用更新            │
│  └─ 算 ρ，調 λ                    │
└──────────────────────────────────┘
    ↓ 收斂
讀回 s, Rwg, bg, ba, v
    ↓
ApplyScaledRotation 套用到地圖
    ↓
標記 isImuInitialized = true
    ↓
（FullInertialBA 接力）
```

---
---

# Part 2：FullInertialBA 完整推導（k=2）

接續 InertialOptimization 之後，由 `InitializeIMU` 呼叫 `FullInertialBA`。設定 k=2（2 個 KF：KF₀ 和 KF₁，1 條 EdgeInertial），並假設有 M 個 MapPoint。

**對應程式：** `Optimizer.cc:392-740`

**和 InertialOptimization 的根本差異：**
- KF 位姿是**可優化**的（不再 fixed）
- **沒有** scale s 和重力 Rwg 變數（已套用到地圖）
- 重力是**固定常數** $\mathbf{g} = [0, 0, -9.81]^T$
- IMU edge 改用 `EdgeInertial`（6 頂點，少 Rwg、s）
- 多了 **MapPoint 頂點**和**視覺 reprojection** 邊
- IMU edge 加了 **Huber kernel**（$\delta = \sqrt{16.92}$）

---

## Step 1：初始值來自前一階段

FullInertialBA **本身不計算初始值**，所有變數都是從 InertialOptimization 後的結果繼承：

| 變數 | 初始值來源 |
|------|-----------|
| $\mathbf{R}_{wb_i}, \mathbf{t}_{wb_i}$ | KF 已被 ApplyScaledRotation 套用 scale 和 Rwg |
| $\mathbf{v}_i$ | InertialOptimization 寫回，再經 ApplyScaledRotation 套用 |
| $\mathbf{b}^g, \mathbf{b}^a$ | InertialOptimization 寫回 |
| MapPoint $\mathbf{p}_j$ | 已被 ApplyScaledRotation 套用 |

---

## Step 2：FullInertialBA 建立 Graph

**對應程式：** `Optimizer.cc:392-602`

### 2.1 LM 求解器設定（line 406-407）

```cpp
g2o::OptimizationAlgorithmLevenberg* solver = ...;
solver->setUserLambdaInit(1e-5);  // 初始 λ = 1e-5
```

注意比 InertialOptimization 的 $\lambda_0 = 10^3$ 小非常多，因為這時已有好的初始值，接近高斯牛頓行為比較合適。

### 2.2 頂點清單

假設 `bInit=true`（從 InitializeIMU 呼叫，priorA ≠ 0）：

| 頂點 | 維度 | 初始值 | fixed? |
|------|------|--------|--------|
| `VertexPose` VP₀ | 6 | $\mathbf{R}_{wb_0}, \mathbf{t}_{wb_0}$ | **false**（可優化）|
| `VertexPose` VP₁ | 6 | $\mathbf{R}_{wb_1}, \mathbf{t}_{wb_1}$ | **false** |
| `VertexVelocity` VV₀ | 3 | $\mathbf{v}_0$ | false |
| `VertexVelocity` VV₁ | 3 | $\mathbf{v}_1$ | false |
| `VertexGyroBias` VG | 3 | 繼承 bg（**所有 KF 共用一個**）| false |
| `VertexAccBias` VA | 3 | 繼承 ba（**共用**）| false |
| `VertexSBAPointXYZ` VPoint_j × M | 3 | $\mathbf{p}_j^{world}$ | false（marginalized）|

優化變數共 **6+6+3+3+3+3+3M = 24+3M 維**。

### 2.3 邊清單

**EdgeInertial**（line 539-544，setRobustKernel 在 541-542，1 條）：
```
連接：VP₀, VV₀, VG, VA, VP₁, VV₁（6 頂點）
information = (C[0:9, 0:9])^{-1}
RobustKernelHuber，delta = sqrt(16.92)
```

**EdgePriorGyro**（line 587-591，bInit=true 時加）：
```
連接：VG
information = priorG · I₃
```

**EdgePriorAcc**（line 580-584，bInit=true 時加）：
```
連接：VA
information = priorA · I₃
```

**EdgeMono / EdgeStereo**（line 633-720，每個觀測一條）：
```
連接：VPoint_j, VP_i（2 個頂點）
EdgeMono：information = invSigma2 · I₂，Huber delta = sqrt(5.991)
EdgeStereo：information = invSigma2 · I₃，Huber delta = sqrt(7.815)
```

註：bInit=true 時**沒有** EdgeGyroRW/EdgeAccRW（line 555-571 的程式碼只在 `!bInit` 時執行）。

---

## Step 3：computeError()

### 3.1 EdgeInertial（G2oTypes.cc:514-535）

**Bias 修正**（同前）：

$$\Delta\mathbf{R}(\mathbf{b}_0) = \bar{\Delta\mathbf{R}} \cdot \text{Exp}(\mathbf{J}^R_g \cdot \delta\mathbf{b}^g)$$

$$\Delta\mathbf{v}(\mathbf{b}_0) = \bar{\Delta\mathbf{v}} + \mathbf{J}^v_g \cdot \delta\mathbf{b}^g + \mathbf{J}^v_a \cdot \delta\mathbf{b}^a$$

$$\Delta\mathbf{p}(\mathbf{b}_0) = \bar{\Delta\mathbf{p}} + \mathbf{J}^p_g \cdot \delta\mathbf{b}^g + \mathbf{J}^p_a \cdot \delta\mathbf{b}^a$$

注意：bias 是 KF₀ 的（前一個 KF），不是當前 KF。

**重力**（line 498，固定常數）：

$$\mathbf{g} = [0, 0, -9.81]^T$$

**旋轉殘差**（line 530，與 InertialOptimization 完全相同）：

$$\mathbf{r}_{\Delta R} = \text{Log}\!\left(\Delta\mathbf{R}(\mathbf{b}_0)^T \cdot \mathbf{R}_{wb_0}^T \cdot \mathbf{R}_{wb_1}\right)$$

**速度殘差**（line 531，**沒有 scale s**）：

$$\mathbf{r}_{\Delta v} = \mathbf{R}_{wb_0}^T \left(\mathbf{v}_1 - \mathbf{v}_0 - \mathbf{g}\Delta t\right) - \Delta\mathbf{v}(\mathbf{b}_0)$$

**位置殘差**（line 532，**沒有 scale s**）：

$$\mathbf{r}_{\Delta p} = \mathbf{R}_{wb_0}^T \left(\mathbf{t}_{wb_1} - \mathbf{t}_{wb_0} - \mathbf{v}_0 \Delta t - \frac{1}{2}\mathbf{g}\Delta t^2\right) - \Delta\mathbf{p}(\mathbf{b}_0)$$

合併成 9 維：

$$\mathbf{r}_{\mathcal{I}} = [\mathbf{r}_{\Delta R}, \mathbf{r}_{\Delta v}, \mathbf{r}_{\Delta p}]^T \in \mathbb{R}^9$$

### 3.2 EdgePriorGyro / EdgePriorAcc（同前）

$$\mathbf{r}_{pg} = \mathbf{0} - \mathbf{b}^g, \quad \mathbf{r}_{pa} = \mathbf{0} - \mathbf{b}^a$$

### 3.3 EdgeMono（G2oTypes.h:353-356）

對 KF $i$ 觀測 MapPoint $j$，量測像素 $\mathbf{u}_{obs} = (u, v)^T$：

$$\mathbf{r}_{\text{mono}, ij} = \mathbf{u}_{obs} - \mathbf{\Pi}\!\left(\mathbf{T}_{cw_i} \cdot \mathbf{p}_j\right) \in \mathbb{R}^2$$

其中 $\mathbf{\Pi}$ 是相機投影函式（針孔或魚眼），$\mathbf{T}_{cw_i} = \mathbf{T}_{cb} \cdot \mathbf{T}_{wb_i}^{-1}$（從 VertexPose 算出）。

展開：設 $\mathbf{X}_c = \mathbf{R}_{cw_i} \mathbf{p}_j + \mathbf{t}_{cw_i}$（point in camera frame），

$$\mathbf{\Pi}(\mathbf{X}_c) = \begin{bmatrix} f_x \cdot X_c^x / X_c^z + c_x \\ f_y \cdot X_c^y / X_c^z + c_y \end{bmatrix}$$

### 3.4 EdgeStereo（G2oTypes.h:435-438）

$$\mathbf{r}_{\text{stereo}, ij} = \begin{bmatrix} u \\ v \\ u_r \end{bmatrix}_{obs} - \mathbf{\Pi}_{\text{stereo}}\!\left(\mathbf{T}_{cw_i} \cdot \mathbf{p}_j\right) \in \mathbb{R}^3$$

其中 $u_r = u - \frac{f_x \cdot b}{X_c^z}$（baseline 視差），$b$ 是 stereo baseline。

### 3.5 Total cost

$$C = \rho_{\text{Hub}}\!\left(\mathbf{r}_{\mathcal{I}}^T \mathbf{\Omega}_I \mathbf{r}_{\mathcal{I}}\right) + \mathbf{r}_{pg}^T (\text{priorG} \cdot I_3) \mathbf{r}_{pg} + \mathbf{r}_{pa}^T (\text{priorA} \cdot I_3) \mathbf{r}_{pa} + \sum_{ij} \rho_{\text{Hub}}\!\left(\mathbf{r}_{\text{vis},ij}^T \mathbf{\Omega}_{ij} \mathbf{r}_{\text{vis},ij}\right)$$

其中 $\rho_{\text{Hub}}(s)$ 是 Huber 函式：

$$\rho_{\text{Hub}}(s) = \begin{cases} s & s \leq \delta^2 \\ 2\delta\sqrt{s} - \delta^2 & s > \delta^2 \end{cases}$$

---

## Step 4：linearizeOplus()

優化變數順序（k=2，bInit=true）：

$$\mathbf{x} = [\underbrace{\mathbf{T}_0}_{6}, \underbrace{\mathbf{T}_1}_{6}, \underbrace{\mathbf{v}_0}_{3}, \underbrace{\mathbf{v}_1}_{3}, \underbrace{\mathbf{b}^g}_{3}, \underbrace{\mathbf{b}^a}_{3}, \underbrace{\mathbf{p}_1, ..., \mathbf{p}_M}_{3M}]$$

維度：24 + 3M。

### 4.1 EdgeInertial Jacobian（9 × 24，不含 MapPoint）

中間量：
- $\mathbf{R}_{bw_0} = \mathbf{R}_{wb_0}^T$
- $\mathbf{e}_R = \Delta\mathbf{R}(\mathbf{b}_0)^T \mathbf{R}_{bw_0} \mathbf{R}_{wb_1}$
- $\mathbf{J}_r^{-1} = \text{InverseRightJacobianSO3}(\text{Log}(\mathbf{e}_R))$

**對 $\mathbf{T}_0$**（6 欄，line 558-567）：

VertexPose 的更新是 $(\delta\phi, \delta\rho)$，即旋轉 3 + 平移 3。

$$\frac{\partial \mathbf{r}_{\Delta R}}{\partial \delta\phi_0} = -\mathbf{J}_r^{-1} \mathbf{R}_{wb_1}^T \mathbf{R}_{wb_0}, \quad \frac{\partial \mathbf{r}_{\Delta R}}{\partial \delta\rho_0} = \mathbf{0}$$

$$\frac{\partial \mathbf{r}_{\Delta v}}{\partial \delta\phi_0} = [\mathbf{R}_{bw_0}(\mathbf{v}_1 - \mathbf{v}_0 - \mathbf{g}\Delta t)]_\times, \quad \frac{\partial \mathbf{r}_{\Delta v}}{\partial \delta\rho_0} = \mathbf{0}$$

$$\frac{\partial \mathbf{r}_{\Delta p}}{\partial \delta\phi_0} = [\mathbf{R}_{bw_0}(\mathbf{t}_{wb_1} - \mathbf{t}_{wb_0} - \mathbf{v}_0\Delta t - \tfrac{1}{2}\mathbf{g}\Delta t^2)]_\times$$

$$\frac{\partial \mathbf{r}_{\Delta p}}{\partial \delta\rho_0} = -\mathbf{I}_3$$

（$[\cdot]_\times$ 是 hat operator）

**對 $\mathbf{T}_1$**（6 欄，line 585-590）：

$$\frac{\partial \mathbf{r}_{\Delta R}}{\partial \delta\phi_1} = \mathbf{J}_r^{-1}, \quad \frac{\partial \mathbf{r}_{\Delta R}}{\partial \delta\rho_1} = \mathbf{0}$$

$$\frac{\partial \mathbf{r}_{\Delta v}}{\partial \delta\phi_1} = \mathbf{0}, \quad \frac{\partial \mathbf{r}_{\Delta v}}{\partial \delta\rho_1} = \mathbf{0}$$

$$\frac{\partial \mathbf{r}_{\Delta p}}{\partial \delta\phi_1} = \mathbf{0}, \quad \frac{\partial \mathbf{r}_{\Delta p}}{\partial \delta\rho_1} = \mathbf{R}_{bw_0} \mathbf{R}_{wb_1}$$

**對 $\mathbf{v}_0$**（3 欄，line 569-572）：

$$\frac{\partial \mathbf{r}_{\Delta R}}{\partial \mathbf{v}_0} = \mathbf{0}, \quad \frac{\partial \mathbf{r}_{\Delta v}}{\partial \mathbf{v}_0} = -\mathbf{R}_{bw_0}, \quad \frac{\partial \mathbf{r}_{\Delta p}}{\partial \mathbf{v}_0} = -\mathbf{R}_{bw_0} \Delta t$$

**對 $\mathbf{v}_1$**（3 欄，line 593-594）：

$$\frac{\partial \mathbf{r}_{\Delta R}}{\partial \mathbf{v}_1} = \mathbf{0}, \quad \frac{\partial \mathbf{r}_{\Delta v}}{\partial \mathbf{v}_1} = \mathbf{R}_{bw_0}, \quad \frac{\partial \mathbf{r}_{\Delta p}}{\partial \mathbf{v}_1} = \mathbf{0}$$

**對 $\mathbf{b}^g$**（3 欄，line 575-578）：

$$\frac{\partial \mathbf{r}_{\Delta R}}{\partial \mathbf{b}^g} = -\mathbf{J}_r^{-1} \mathbf{e}_R^T \mathbf{J}_r(\mathbf{J}^R_g \delta\mathbf{b}^g) \mathbf{J}^R_g$$

$$\frac{\partial \mathbf{r}_{\Delta v}}{\partial \mathbf{b}^g} = -\mathbf{J}^v_g, \quad \frac{\partial \mathbf{r}_{\Delta p}}{\partial \mathbf{b}^g} = -\mathbf{J}^p_g$$

**對 $\mathbf{b}^a$**（3 欄，line 580-583）：

$$\frac{\partial \mathbf{r}_{\Delta R}}{\partial \mathbf{b}^a} = \mathbf{0}, \quad \frac{\partial \mathbf{r}_{\Delta v}}{\partial \mathbf{b}^a} = -\mathbf{J}^v_a, \quad \frac{\partial \mathbf{r}_{\Delta p}}{\partial \mathbf{b}^a} = -\mathbf{J}^p_a$$

EdgeInertial 對 MapPoint 都是 0。

### 4.2 完整 J 矩陣（EdgeInertial 9×24，block 形式）

$$\mathbf{J}_{EI} = \begin{bmatrix}
-\mathbf{J}_r^{-1}\mathbf{R}_{wb_1}^T\mathbf{R}_{wb_0} & \mathbf{0}_{3\times3} & \mathbf{J}_r^{-1} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & -\mathbf{J}_r^{-1}\mathbf{e}_R^T\mathbf{J}_r\mathbf{J}^R_g & \mathbf{0}_{3\times3} \\
[\mathbf{R}_{bw_0}(\mathbf{v}_1-\mathbf{v}_0-\mathbf{g}\Delta t)]_\times & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & -\mathbf{R}_{bw_0} & \mathbf{R}_{bw_0} & -\mathbf{J}^v_g & -\mathbf{J}^v_a \\
[\mathbf{R}_{bw_0}(\mathbf{t}_1-\mathbf{t}_0-\mathbf{v}_0\Delta t-\tfrac{1}{2}\mathbf{g}\Delta t^2)]_\times & -\mathbf{I}_3 & \mathbf{0}_{3\times3} & \mathbf{R}_{bw_0}\mathbf{R}_{wb_1} & -\mathbf{R}_{bw_0}\Delta t & \mathbf{0}_{3\times3} & -\mathbf{J}^p_g & -\mathbf{J}^p_a
\end{bmatrix}$$

欄位順序：$[\boldsymbol{\phi}_0, \boldsymbol{\rho}_0, \boldsymbol{\phi}_1, \boldsymbol{\rho}_1, \mathbf{v}_0, \mathbf{v}_1, \mathbf{b}^g, \mathbf{b}^a]$，列順序：$[\mathbf{r}_{\Delta R}, \mathbf{r}_{\Delta v}, \mathbf{r}_{\Delta p}]$。

對 MapPoint $\mathbf{p}_j$ 全部是 0，沒列出來。

### 4.3 EdgeMono Jacobian（2 × 9，G2oTypes.cc:349-372）

對 MapPoint 和 Pose：

中間量：
- $\mathbf{X}_c = \mathbf{R}_{cw} \mathbf{p}_j + \mathbf{t}_{cw}$（相機座標）
- $\mathbf{X}_b = \mathbf{R}_{bc} \mathbf{X}_c + \mathbf{t}_{bc}$（body 座標）
- $\mathbf{J}_{\text{proj}} = \frac{\partial \mathbf{\Pi}}{\partial \mathbf{X}_c}\Big|_{\mathbf{X}_c}$（2×3 投影 Jacobian）

對於針孔模型：

$$\mathbf{J}_{\text{proj}} = \begin{bmatrix} f_x / Z & 0 & -f_x X / Z^2 \\ 0 & f_y / Z & -f_y Y / Z^2 \end{bmatrix}$$

**對 MapPoint $\mathbf{p}_j$**（line 359）：

$$\frac{\partial \mathbf{r}_{\text{mono}}}{\partial \mathbf{p}_j} = -\mathbf{J}_{\text{proj}} \cdot \mathbf{R}_{cw}$$

**對 Pose $\mathbf{T}_i$**（line 361-372）：

定義 SE3 導數（在 body frame 下）：

$$\mathbf{S}_{SE3} = \begin{bmatrix} 0 & z & -y & 1 & 0 & 0 \\ -z & 0 & x & 0 & 1 & 0 \\ y & -x & 0 & 0 & 0 & 1 \end{bmatrix}, \quad (x, y, z) = \mathbf{X}_b$$

$$\frac{\partial \mathbf{r}_{\text{mono}}}{\partial \mathbf{T}_i} = \mathbf{J}_{\text{proj}} \cdot \mathbf{R}_{cb} \cdot \mathbf{S}_{SE3}$$

### 4.4 EdgeStereo Jacobian（3 × 9，類似）

多一維（$u_r$），投影 Jacobian 是 3×3：

$$\mathbf{J}_{\text{proj}}^{stereo} = \begin{bmatrix} f_x/Z & 0 & -f_x X/Z^2 \\ 0 & f_y/Z & -f_y Y/Z^2 \\ f_x/Z & 0 & -f_x X/Z^2 + b f_x / Z^2 \end{bmatrix}$$

其餘結構同 EdgeMono。

### 4.5 Prior Jacobian（bInit=true 才有，G2oTypes.cc:762-775）

EdgePriorGyro / EdgePriorAcc 是 unary edge：

$$\frac{\partial \mathbf{r}_{pg}}{\partial \mathbf{b}^g} = +I_3, \quad \frac{\partial \mathbf{r}_{pa}}{\partial \mathbf{b}^a} = +I_3 \quad \text{（程式實作，理論應為 } -I_3\text{）}$$

VIBA 2（bInit=false）不加 Prior 邊，這個 Jacobian 不存在。

### 4.6 Bias Random Walk Jacobian（bInit=false 才有，G2oTypes.h:645、681）

當 bInit=false 時，每個 KF 各有 bias，多兩條 binary edge 約束相鄰 KF 的 bias 連續性。

殘差：

$$\mathbf{r}_{gRW} = \mathbf{b}^g_1 - \mathbf{b}^g_0, \quad \mathbf{r}_{aRW} = \mathbf{b}^a_1 - \mathbf{b}^a_0$$

Jacobian：

$$\frac{\partial \mathbf{r}_{gRW}}{\partial \mathbf{b}^g_0} = -I_3, \quad \frac{\partial \mathbf{r}_{gRW}}{\partial \mathbf{b}^g_1} = +I_3$$

$$\frac{\partial \mathbf{r}_{aRW}}{\partial \mathbf{b}^a_0} = -I_3, \quad \frac{\partial \mathbf{r}_{aRW}}{\partial \mathbf{b}^a_1} = +I_3$$

```cpp
virtual void linearizeOplus(){
    _jacobianOplusXi = -Eigen::Matrix3d::Identity();
    _jacobianOplusXj.setIdentity();
}
```

### 4.7 J 拆成各邊獨立的 Jacobian

g2o 內部不組成完整 J，每條邊各自存 Jacobian：

| Edge | 連接頂點 | Jacobian 維度 |
|------|---------|--------------|
| EdgeInertial | T0, V0, bg(0), ba(0), T1, V1 | 6 個 block，總 9×24 |
| EdgePriorGyro | bg | 3×3（bInit=true）|
| EdgePriorAcc | ba | 3×3（bInit=true）|
| EdgeGyroRW | bg0, bg1 | 2 個 3×3（bInit=false）|
| EdgeAccRW | ba0, ba1 | 2 個 3×3（bInit=false）|
| EdgeMono | p_j, T_i | 2×3 + 2×6 |
| EdgeStereo | p_j, T_i | 3×3 + 3×6 |

組裝 H 時 g2o 把它們疊加進對應的 block，配合 Schur complement 消去 MapPoint 變數。

---

## Step 5：組裝 H 和 b

### 5.1 完整 H

$$\mathbf{H} = \mathbf{J}_I^T \mathbf{\Omega}_I \mathbf{J}_I + \mathbf{J}_{pg}^T (\text{priorG} I) \mathbf{J}_{pg} + \mathbf{J}_{pa}^T (\text{priorA} I) \mathbf{J}_{pa} + \sum_{ij} \mathbf{J}_{vis,ij}^T \mathbf{\Omega}_{ij} \mathbf{J}_{vis,ij}$$

### 5.2 視覺項的 Schur Complement

由於 MapPoint 數量 M 很大（可能上千），直接解 $(24 + 3M)\times(24 + 3M)$ 的 H 太貴。

g2o 對 MapPoint 頂點呼叫 `setMarginalized(true)`（line 605），在求解時自動做 Schur complement：

把 H 分塊：

$$\mathbf{H} = \begin{bmatrix} \mathbf{H}_{cc} & \mathbf{H}_{cp} \\ \mathbf{H}_{cp}^T & \mathbf{H}_{pp} \end{bmatrix}, \quad \mathbf{b} = \begin{bmatrix} \mathbf{b}_c \\ \mathbf{b}_p \end{bmatrix}$$

其中 $c$ 表示 camera/IMU 變數（24 維），$p$ 表示 MapPoint（3M 維）。

Schur 消元：

$$\left(\mathbf{H}_{cc} - \mathbf{H}_{cp} \mathbf{H}_{pp}^{-1} \mathbf{H}_{cp}^T\right) \delta\mathbf{x}_c = \mathbf{b}_c - \mathbf{H}_{cp} \mathbf{H}_{pp}^{-1} \mathbf{b}_p$$

由於 $\mathbf{H}_{pp}$ 是 block-diagonal（每個 MapPoint 3×3），$\mathbf{H}_{pp}^{-1}$ 很容易算。先解出 $\delta\mathbf{x}_c$，再回代算 $\delta\mathbf{x}_p$：

$$\delta\mathbf{x}_p = \mathbf{H}_{pp}^{-1}(\mathbf{b}_p - \mathbf{H}_{cp}^T \delta\mathbf{x}_c)$$

---

## Step 6：解 (H + λI)δx = -b

### 6.1 λ 初始值

```cpp
solver->setUserLambdaInit(1e-5);  // line 407
```

$\lambda_0 = 10^{-5}$，比 InertialOptimization 小很多，因為這時已有準確初始值，希望快速收斂（接近高斯牛頓）。

### 6.2 解線性方程

$$(\mathbf{H}_{cc}^{Schur} + \lambda \mathbf{I}) \delta\mathbf{x}_c = -\mathbf{b}_c^{Schur}$$

g2o 用 `LinearSolverEigen<BlockSolverX::PoseMatrixType>` 做稀疏 Cholesky 分解。

得到 camera/IMU 變數的更新（k=2、bInit=true，共 24 維）：

$$\delta\mathbf{x}_c = [\underbrace{\delta\mathbf{T}_0}_{6},\ \underbrace{\delta\mathbf{T}_1}_{6},\ \underbrace{\delta\mathbf{v}_0}_{3},\ \underbrace{\delta\mathbf{v}_1}_{3},\ \underbrace{\delta\mathbf{b}^g}_{3},\ \underbrace{\delta\mathbf{b}^a}_{3}] \in \mathbb{R}^{24}$$

其中 $\delta\mathbf{T}_i = (\delta\boldsymbol{\phi}_i, \delta\boldsymbol{\rho}_i)$ 各為 6 維（旋轉 3 + 平移 3）。

接著回代算 MapPoint 更新：

$$\delta\mathbf{x}_p = [\delta\mathbf{p}_1, \delta\mathbf{p}_2, ..., \delta\mathbf{p}_M] \in \mathbb{R}^{3M}$$

整體 $\delta\mathbf{x} = [\delta\mathbf{x}_c; \delta\mathbf{x}_p] \in \mathbb{R}^{24+3M}$。

**bInit=false 的情況**（VIBA 2）：每個 KF 各有 bg、ba，所以 $\delta\mathbf{x}_c$ 變 30 維（k=2 時），整體 $30 + 3M$ 維。

---

## Step 7：oplusImpl()

### 7.1 VertexPose（G2oTypes.h:149-151，ImuCamPose::Update）

設 $\delta\mathbf{T}_i = (\delta\phi, \delta\rho)$：

$$\mathbf{R}_{wb_i}^{\text{new}} = \mathbf{R}_{wb_i}^{\text{old}} \cdot \text{Exp}(\delta\phi)$$

$$\mathbf{t}_{wb_i}^{\text{new}} = \mathbf{t}_{wb_i}^{\text{old}} + \mathbf{R}_{wb_i}^{\text{old}} \cdot \delta\rho$$

每 3 次更新後做 `NormalizeRotation` 確保 $\mathbf{R}_{wb}$ 仍是合法旋轉矩陣（line 204）。

接著更新所有相機 pose（line 213-217）：

$$\mathbf{R}_{cw_i}^{\text{new}} = \mathbf{R}_{cb} \cdot \mathbf{R}_{wb_i}^{T,\text{new}}, \quad \mathbf{t}_{cw_i}^{\text{new}} = -\mathbf{R}_{cw_i}^{\text{new}} \cdot \mathbf{t}_{wb_i}^{\text{new}} + \mathbf{t}_{cb}$$

### 7.2 VertexVelocity（同前）

$$\mathbf{v}_i^{\text{new}} = \mathbf{v}_i^{\text{old}} + \delta\mathbf{v}_i$$

### 7.3 VertexGyroBias / VertexAccBias（同前）

$$\mathbf{b}^{g,\text{new}} = \mathbf{b}^{g,\text{old}} + \delta\mathbf{b}^g, \quad \mathbf{b}^{a,\text{new}} = \mathbf{b}^{a,\text{old}} + \delta\mathbf{b}^a$$

### 7.4 VertexSBAPointXYZ（g2o 內建）

$$\mathbf{p}_j^{\text{new}} = \mathbf{p}_j^{\text{old}} + \delta\mathbf{p}_j$$

---

## Step 8：驗證 cost 並調整 λ

同 InertialOptimization 的規則：

$$\rho = \frac{C^{\text{old}} - C^{\text{new}}}{\delta\mathbf{x}^T(\lambda \delta\mathbf{x} - \mathbf{b})}$$

- $\rho > 0$：接受，$\lambda \cdot \max(1/3, 1 - (2\rho-1)^3)$
- $\rho \leq 0$：拒絕，$\lambda \cdot 2$，重新解

額外注意：**Huber kernel** 會把大 residual 的影響線性化（不是平方），改變實際的 cost 和 Jacobian 加權。每次迭代 g2o 自動處理。

---

## Step 9：重複到收斂

`optimizer.optimize(its)`，`its = 100`（從 InitializeIMU 傳入）。

最多 100 次，提早收斂條件同前。

---

## Step 10：讀回結果

**對應程式：** `Optimizer.cc:734-808`

注意 FullInertialBA 是把優化結果**直接寫回 KF 和 MapPoint**，但有兩種模式：

### 10.1 nLoopId == 0（一般模式，從 InitializeIMU 呼叫）

對每個 KF（line 740-770）：
```cpp
Sophus::SE3f Tcw(VP->estimate().Rcw[0], VP->estimate().tcw[0]);
pKFi->SetPose(Tcw);                      // 寫回位姿
pKFi->SetVelocity(VV->estimate());       // 寫回速度
pKFi->SetNewBias(b);                     // 寫回 bias
```

對每個 MapPoint（line 800-810）：
```cpp
pMP->SetWorldPos(vPoint->estimate());
pMP->UpdateNormalAndDepth();
```

### 10.2 nLoopId != 0（迴圈閉合的 GBA 模式）

寫到備份欄位 `mTcwGBA`、`mVwbGBA`、`mBiasGBA`、`mPosGBA`，等之後 propagate 階段再套用。InitializeIMU 用前者。

---

## Step 11：FullInertialBA 不呼叫 ApplyScaledRotation

**關鍵差異**：FullInertialBA 結束後直接結束，**不會**像 InertialOptimization 那樣呼叫 `ApplyScaledRotation`。原因：

- 沒有 scale 和 Rwg 變數要套用
- 重力是固定常數
- 位姿、速度、MapPoint 已經在 `SetPose`、`SetVelocity`、`SetWorldPos` 步驟直接寫回地圖

整個 FullInertialBA 結束後 `InitializeIMU` 也跟著結束，控制權回到 LocalMapping 主迴圈。

---

## 整體流程總結

```
ApplyScaledRotation 完成 → FullInertialBA 入口
    ↓
建 graph：
  6 個 IMU 頂點（2 KF 各有 Pose+Velocity，共用 bg、ba）
  M 個 MapPoint 頂點（marginalized）
  邊：1 EdgeInertial + 2 Prior + K reprojection
    ↓
┌─────────────────────────────────────────┐
│  LM 迭代 100 次                          │
│  ├─ computeError                        │
│  │    9 維 IMU + 3+3 prior + K 視覺      │
│  ├─ linearizeOplus                      │
│  │    EdgeInertial、EdgeMono/Stereo     │
│  ├─ Schur 消去 MapPoint                 │
│  ├─ 解 (H_cc + λI)δx_c = -b_c           │
│  ├─ 回代算 δx_p                          │
│  ├─ oplusImpl 套用所有更新              │
│  └─ 算 ρ，調 λ                           │
└─────────────────────────────────────────┘
    ↓ 收斂
讀回 Pose, Velocity, Bias, MapPoint
    ↓
SetPose / SetVelocity / SetNewBias / SetWorldPos
    ↓
直接結束（不需要 ApplyScaledRotation）
```

---

## InertialOptimization vs FullInertialBA 對照表

| 項目 | InertialOptimization | FullInertialBA |
|------|---------------------|----------------|
| 目標 | 估出 s, Rwg, b, v 的初始值 | 加入視覺後完整 BA |
| KF 位姿 | 固定 | **可優化** |
| scale s | 優化變數 | 不存在 |
| 重力 Rwg | 優化變數（藏在 g 裡）| 不存在，g = 固定常數 |
| bias | 共用一組 | bInit=true 共用 / bInit=false 每 KF |
| MapPoint | 不參與 | **參與優化（marginalized）**|
| IMU edge | EdgeInertialGS（8 頂點）| EdgeInertial（6 頂點）|
| 視覺 edge | 無 | **EdgeMono / EdgeStereo** |
| Bias RW edge | 無 | bInit=false 才有 EdgeGyroRW/AccRW |
| Robust kernel（IMU）| 無 | Huber，δ = √16.92 |
| LM 初始 λ | 1e3（priorG≠0）| 1e-5 |
| 迭代次數 | 200 | 100 |
| 結束後處理 | ApplyScaledRotation | 直接結束 |

---
---

# Appendix：J 矩陣完整展開（每個元素都列出）

## A.1 InertialOptimization 的 J 矩陣（k=2，15×15 = 225 個元素）

**符號定義**

15 個變數對應的欄位：

| 欄 | 變數 |
|----|------|
| 0 | v0_x |
| 1 | v0_y |
| 2 | v0_z |
| 3 | v1_x |
| 4 | v1_y |
| 5 | v1_z |
| 6 | bg_x |
| 7 | bg_y |
| 8 | bg_z |
| 9 | ba_x |
| 10 | ba_y |
| 11 | ba_z |
| 12 | α |
| 13 | β |
| 14 | s |

15 列對應的殘差：

| 列 | 殘差分量 |
|----|----------|
| 0 | r_ΔR_x |
| 1 | r_ΔR_y |
| 2 | r_ΔR_z |
| 3 | r_Δv_x |
| 4 | r_Δv_y |
| 5 | r_Δv_z |
| 6 | r_Δp_x |
| 7 | r_Δp_y |
| 8 | r_Δp_z |
| 9 | r_pg_x |
| 10 | r_pg_y |
| 11 | r_pg_z |
| 12 | r_pa_x |
| 13 | r_pa_y |
| 14 | r_pa_z |

**矩陣記號縮寫**

設 `R = Rbw0`，每個元素用 `R_ij` 表示矩陣第 i 列第 j 欄。

- `(Jbg_R) = -Jr⁻¹·eR^T·Jr(JRg·δbg)·JRg` ∈ ℝ^{3×3}
- `(gv) = -R·Rwg·Gm·Δt` ∈ ℝ^{3×2}
- `(gp) = -½·R·Rwg·Gm·Δt²` ∈ ℝ^{3×2}
- `(sv) = R·(v1-v0)` ∈ ℝ^{3×1}
- `(sp) = R·(t1-t0-v0·Δt)` ∈ ℝ^{3×1}

**完整 J 矩陣（LaTeX bmatrix，每個元素都展開）**

設 $A_{ij} = (\mathbf{J}_{bg,R})_{ij}$（Jbg_R 的元素），其餘符號照定義。

由於 15×15 太寬，分成五個 row-block 各自寫 3×15 的 sub-matrix。

### 列 0-2（r_ΔR）

$$\mathbf{J}_{[0:3,\ :]} = \begin{bmatrix}
0 & 0 & 0 & 0 & 0 & 0 & A_{00} & A_{01} & A_{02} & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & A_{10} & A_{11} & A_{12} & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & A_{20} & A_{21} & A_{22} & 0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}$$

### 列 3-5（r_Δv）

$$\mathbf{J}_{[3:6,\ :]} = \begin{bmatrix}
-sR_{00} & -sR_{01} & -sR_{02} & sR_{00} & sR_{01} & sR_{02} & -JV^g_{00} & -JV^g_{01} & -JV^g_{02} & -JV^a_{00} & -JV^a_{01} & -JV^a_{02} & gv_{00} & gv_{01} & sv_0 \\
-sR_{10} & -sR_{11} & -sR_{12} & sR_{10} & sR_{11} & sR_{12} & -JV^g_{10} & -JV^g_{11} & -JV^g_{12} & -JV^a_{10} & -JV^a_{11} & -JV^a_{12} & gv_{10} & gv_{11} & sv_1 \\
-sR_{20} & -sR_{21} & -sR_{22} & sR_{20} & sR_{21} & sR_{22} & -JV^g_{20} & -JV^g_{21} & -JV^g_{22} & -JV^a_{20} & -JV^a_{21} & -JV^a_{22} & gv_{20} & gv_{21} & sv_2
\end{bmatrix}$$

### 列 6-8（r_Δp）

$$\mathbf{J}_{[6:9,\ :]} = \begin{bmatrix}
-sR_{00}\Delta t & -sR_{01}\Delta t & -sR_{02}\Delta t & 0 & 0 & 0 & -JP^g_{00} & -JP^g_{01} & -JP^g_{02} & -JP^a_{00} & -JP^a_{01} & -JP^a_{02} & gp_{00} & gp_{01} & sp_0 \\
-sR_{10}\Delta t & -sR_{11}\Delta t & -sR_{12}\Delta t & 0 & 0 & 0 & -JP^g_{10} & -JP^g_{11} & -JP^g_{12} & -JP^a_{10} & -JP^a_{11} & -JP^a_{12} & gp_{10} & gp_{11} & sp_1 \\
-sR_{20}\Delta t & -sR_{21}\Delta t & -sR_{22}\Delta t & 0 & 0 & 0 & -JP^g_{20} & -JP^g_{21} & -JP^g_{22} & -JP^a_{20} & -JP^a_{21} & -JP^a_{22} & gp_{20} & gp_{21} & sp_2
\end{bmatrix}$$

### 列 9-11（r_pg）

$$\mathbf{J}_{[9:12,\ :]} = \begin{bmatrix}
0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}$$

### 列 12-14（r_pa）

$$\mathbf{J}_{[12:15,\ :]} = \begin{bmatrix}
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0
\end{bmatrix}$$

### 完整 15×15 J 矩陣（block 形式）

把上面五個 sub-matrix 縱向疊起來就是完整的 15×15 J：

$$\mathbf{J} = \begin{bmatrix}
\mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{A} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 2} & \mathbf{0}_{3\times 1} \\
-s\mathbf{R} & s\mathbf{R} & -\mathbf{J}^v_g & -\mathbf{J}^v_a & \mathbf{gv} & \mathbf{sv} \\
-s\mathbf{R}\Delta t & \mathbf{0}_{3\times 3} & -\mathbf{J}^p_g & -\mathbf{J}^p_a & \mathbf{gp} & \mathbf{sp} \\
\mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{I}_3 & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 2} & \mathbf{0}_{3\times 1} \\
\mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{I}_3 & \mathbf{0}_{3\times 2} & \mathbf{0}_{3\times 1}
\end{bmatrix}_{15 \times 15}$$

**統計**：3+3+3+3+3 列 × 15 欄 = 225 個元素，全部列出。

**零元素的位置**（共 130 個）：
- r_ΔR：除了 bg 的 9 個元素，其他 36 個都是 0
- r_Δv：全部 45 個都非零
- r_Δp：v1 的 9 個元素是 0，共 9 個 0
- r_pg：除了 bg 對角線的 3 個 1，其他 42 個都是 0
- r_pa：除了 ba 對角線的 3 個 1，其他 42 個都是 0

非零元素總數：9 (r_ΔR) + 45 (r_Δv) + 36 (r_Δp) + 3 (r_pg) + 3 (r_pa) = **96 個非零元素**

---

**設定**：k=2（2 個 KF），M=1（1 個 MapPoint），1 個視覺觀測（KF₀ 觀測 p₁）。

**變數順序**（共 27 維）：

| 欄 | 變數 |
|----|------|
| 0 | φ0_x（T0 旋轉 x） |
| 1 | φ0_y |
| 2 | φ0_z |
| 3 | ρ0_x（T0 平移 x） |
| 4 | ρ0_y |
| 5 | ρ0_z |
| 6 | φ1_x |
| 7 | φ1_y |
| 8 | φ1_z |
| 9 | ρ1_x |
| 10 | ρ1_y |
| 11 | ρ1_z |
| 12 | v0_x |
| 13 | v0_y |
| 14 | v0_z |
| 15 | v1_x |
| 16 | v1_y |
| 17 | v1_z |
| 18 | bg_x |
| 19 | bg_y |
| 20 | bg_z |
| 21 | ba_x |
| 22 | ba_y |
| 23 | ba_z |
| 24 | p1_x |
| 25 | p1_y |
| 26 | p1_z |

**殘差列**（共 17 列）：

| 列 | 殘差分量 |
|----|----------|
| 0-2 | r_ΔR |
| 3-5 | r_Δv |
| 6-8 | r_Δp |
| 9-11 | r_pg |
| 12-14 | r_pa |
| 15-16 | r_uv（視覺，2 維） |

**矩陣記號縮寫**

- `R = Rbw0 = Rwb0^T`
- `R₂ = Rbw0·Rwb1`
- `(JR_T0)_ij = -Jr⁻¹·Rwb1^T·Rwb0` 第 i 列第 j 欄
- `(Jr⁻¹)_ij`：InverseRightJacobian 矩陣元素
- `(Jbg_R)_ij = (-Jr⁻¹·eR^T·Jr(JRg·δbg)·JRg)_ij`
- `(Jv_T0)_ij = [R(v1-v0-g·Δt)]×_ij`（hat operator）
- `(Jp_T0)_ij = [R(t1-t0-v0·Δt-½g·Δt²)]×_ij`
- `JVg_ij, JVa_ij, JPg_ij, JPa_ij`：bias Jacobian 元素
- `(Jpose)_ij = (Jproj·Rcb·S_SE3)_ij`（2×6，對 T0）
- `(Jpoint)_ij = (-Jproj·Rcw0)_ij`（2×3，對 p1）

由於完整 17×27 = 459 個元素太寬，每個 row-block 拆成 LaTeX bmatrix。

**符號縮寫**（每個都是 3×3 或 2×6 等矩陣的元素）：

- $B_{ij} = (\mathbf{JR}_{T0})_{ij} = (-\mathbf{J}_r^{-1} \mathbf{R}_{wb_1}^T \mathbf{R}_{wb_0})_{ij}$
- $C_{ij} = (\mathbf{J}_r^{-1})_{ij}$
- $D_{ij} = (\mathbf{Jv}_{T0})_{ij} = ([\mathbf{R}(\mathbf{v}_1 - \mathbf{v}_0 - \mathbf{g}\Delta t)]_\times)_{ij}$
- $E_{ij} = (\mathbf{Jp}_{T0})_{ij} = ([\mathbf{R}(\mathbf{t}_1 - \mathbf{t}_0 - \mathbf{v}_0\Delta t - \tfrac{1}{2}\mathbf{g}\Delta t^2)]_\times)_{ij}$
- $A_{ij} = (\mathbf{J}_{bg,R})_{ij}$
- $R_{ij} = (\mathbf{R}_{bw_0})_{ij}$
- $R^2_{ij} = (\mathbf{R}_{bw_0} \mathbf{R}_{wb_1})_{ij}$
- $P_{ij} = (\mathbf{J}_{pose})_{ij}$（2×6，Jproj·Rcb·S_SE3）
- $Q_{ij} = (\mathbf{J}_{point})_{ij}$（2×3，-Jproj·Rcw0）

### A.2.1 列 0-2（r_ΔR）

$$\mathbf{J}_{[0:3,\ :]} = \begin{bmatrix}
B_{00} & B_{01} & B_{02} & 0 & 0 & 0 & C_{00} & C_{01} & C_{02} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & A_{00} & A_{01} & A_{02} & 0 & 0 & 0 & 0 & 0 & 0 \\
B_{10} & B_{11} & B_{12} & 0 & 0 & 0 & C_{10} & C_{11} & C_{12} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & A_{10} & A_{11} & A_{12} & 0 & 0 & 0 & 0 & 0 & 0 \\
B_{20} & B_{21} & B_{22} & 0 & 0 & 0 & C_{20} & C_{21} & C_{22} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & A_{20} & A_{21} & A_{22} & 0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}$$

### A.2.2 列 3-5（r_Δv）

$$\mathbf{J}_{[3:6,\ :]} = \begin{bmatrix}
D_{00} & D_{01} & D_{02} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & -R_{00} & -R_{01} & -R_{02} & R_{00} & R_{01} & R_{02} & -JV^g_{00} & -JV^g_{01} & -JV^g_{02} & -JV^a_{00} & -JV^a_{01} & -JV^a_{02} & 0 & 0 & 0 \\
D_{10} & D_{11} & D_{12} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & -R_{10} & -R_{11} & -R_{12} & R_{10} & R_{11} & R_{12} & -JV^g_{10} & -JV^g_{11} & -JV^g_{12} & -JV^a_{10} & -JV^a_{11} & -JV^a_{12} & 0 & 0 & 0 \\
D_{20} & D_{21} & D_{22} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & -R_{20} & -R_{21} & -R_{22} & R_{20} & R_{21} & R_{22} & -JV^g_{20} & -JV^g_{21} & -JV^g_{22} & -JV^a_{20} & -JV^a_{21} & -JV^a_{22} & 0 & 0 & 0
\end{bmatrix}$$

### A.2.3 列 6-8（r_Δp）

$$\mathbf{J}_{[6:9,\ :]} = \begin{bmatrix}
E_{00} & E_{01} & E_{02} & -1 & 0 & 0 & 0 & 0 & 0 & R^2_{00} & R^2_{01} & R^2_{02} & -R_{00}\Delta t & -R_{01}\Delta t & -R_{02}\Delta t & 0 & 0 & 0 & -JP^g_{00} & -JP^g_{01} & -JP^g_{02} & -JP^a_{00} & -JP^a_{01} & -JP^a_{02} & 0 & 0 & 0 \\
E_{10} & E_{11} & E_{12} & 0 & -1 & 0 & 0 & 0 & 0 & R^2_{10} & R^2_{11} & R^2_{12} & -R_{10}\Delta t & -R_{11}\Delta t & -R_{12}\Delta t & 0 & 0 & 0 & -JP^g_{10} & -JP^g_{11} & -JP^g_{12} & -JP^a_{10} & -JP^a_{11} & -JP^a_{12} & 0 & 0 & 0 \\
E_{20} & E_{21} & E_{22} & 0 & 0 & -1 & 0 & 0 & 0 & R^2_{20} & R^2_{21} & R^2_{22} & -R_{20}\Delta t & -R_{21}\Delta t & -R_{22}\Delta t & 0 & 0 & 0 & -JP^g_{20} & -JP^g_{21} & -JP^g_{22} & -JP^a_{20} & -JP^a_{21} & -JP^a_{22} & 0 & 0 & 0
\end{bmatrix}$$

### A.2.4 列 9-11（r_pg）

$$\mathbf{J}_{[9:12,\ :]} = \begin{bmatrix}
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}$$

### A.2.5 列 12-14（r_pa）

$$\mathbf{J}_{[12:15,\ :]} = \begin{bmatrix}
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0
\end{bmatrix}$$

### A.2.6 列 15-16（r_uv，視覺）

$$\mathbf{J}_{[15:17,\ :]} = \begin{bmatrix}
P_{00} & P_{01} & P_{02} & P_{03} & P_{04} & P_{05} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & Q_{00} & Q_{01} & Q_{02} \\
P_{10} & P_{11} & P_{12} & P_{13} & P_{14} & P_{15} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & Q_{10} & Q_{11} & Q_{12}
\end{bmatrix}$$

### A.2.7 完整 17×27 J 矩陣（block 形式）

$$\mathbf{J} = \begin{bmatrix}
\mathbf{B} & \mathbf{0}_{3\times 3} & \mathbf{C} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{A} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} \\
\mathbf{D} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & -\mathbf{R} & \mathbf{R} & -\mathbf{J}^v_g & -\mathbf{J}^v_a & \mathbf{0}_{3\times 3} \\
\mathbf{E} & -\mathbf{I}_3 & \mathbf{0}_{3\times 3} & \mathbf{R}^2 & -\mathbf{R}\Delta t & \mathbf{0}_{3\times 3} & -\mathbf{J}^p_g & -\mathbf{J}^p_a & \mathbf{0}_{3\times 3} \\
\mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{I}_3 & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} \\
\mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{I}_3 & \mathbf{0}_{3\times 3} \\
\mathbf{P}_L & \mathbf{P}_R & \mathbf{0}_{2\times 3} & \mathbf{0}_{2\times 3} & \mathbf{0}_{2\times 3} & \mathbf{0}_{2\times 3} & \mathbf{0}_{2\times 3} & \mathbf{0}_{2\times 3} & \mathbf{Q}
\end{bmatrix}_{17 \times 27}$$

其中 $\mathbf{P}_L$、$\mathbf{P}_R$ 分別是 $\mathbf{J}_{pose}$ 的左 3 欄和右 3 欄。

### A.2.7 統計

- 總元素數：17 × 27 = **459 個**
- r_ΔR：A 區 18 個（9 非零）+ B 區 36 個（9 非零）+ C 區 9 個（0）= 9+9 = 18 個非零
- r_Δv：A 區 18 個（9 非零）+ B 區 36 個（36 非零）+ C 區 9 個（0）= 9+36 = 45 個非零
- r_Δp：A 區 18 個（12 非零，含 -I3 三個 -1）+ B 區 36 個（27 非零）+ C 區 9 個（0）= 12+27 = 39 個非零
- r_pg：3 個非零（對角線）
- r_pa：3 個非零（對角線）
- r_uv：A 區 12 個非零 + C 區 6 個非零 = 18 個非零

**總非零元素：18+45+39+3+3+18 = 126 個**

**零元素：459 - 126 = 333 個**（佔 72.5%）

這就是為什麼需要稀疏求解器（g2o `LinearSolverEigen` 配 `BlockSolverX`）和 Schur complement 來高效求解。

### A.2.8 Schur complement 後的 reduced H

把 MapPoint 變數（p1）消去後，剩下 24 個變數的 reduced system：

```
              T0(6)  T1(6)  v0(3)  v1(3)  bg(3)  ba(3)
            ┌                                          ┐
H_cc^Schur =│         24 × 24 dense (or near-dense)     │
            └                                          ┘
```

MapPoint 被消去後，IMU 變數之間透過視覺項變得相關（fill-in），這個矩陣大致是滿的，但維度只有 24，可以用 Cholesky 直接解。

