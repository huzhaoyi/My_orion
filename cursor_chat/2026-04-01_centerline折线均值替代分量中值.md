# centerline_arclength：折线 3D 窗口均值 + 保守默认

## 原因

分量 **x/y/z 各自中值** 会拼出不一定在杆上的点；z 近似常数时易表现为 **x/y 漂、z 不变**。

## 改动

- **`polyline_window_mean`**：在 PCA 排序后的折线上，对每个顶点用窗口内顶点的 **3D 算术均值**（真实点的凸组合）。
- **默认 `centerline_dedupe_radius_m`**：**0**（不去重）；需要时再设 **0.005～0.03**。
- **默认 `centerline_median_window`**：**1**（不平滑）；需要抑制噪声时再设 **3、5** 等奇数。参数名仍叫 `centerline_median_window`，语义为沿折线平滑窗宽。

## 构建

`colcon build --packages-select orion_mtc` 已通过。
