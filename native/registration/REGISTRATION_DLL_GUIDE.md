# CAD 点云配准 DLL 工业使用与参数指南

## 1. 版本与适用范围

- DLL ABI：`CADREG_ABI_VERSION = 5`
- 内部依赖：PCL 1.14.1
- 编译环境：Visual Studio 2022，x64，Release
- 公共接口：纯 C ABI，不暴露 PCL、Eigen 或 STL 类型
- 变换方向：`p_target = T_source_to_target * p_source`
- 矩阵格式：4×4、行主序
- 当前精配算法：多级 point-to-point ICP

本版本适合型号固定、工位受控、单位一致、扫描点云已去除背景，并经过多批
样件验证的 CAD—扫描点云配准。局部扫描面对长条、对称或重复结构时可能存在
多个几何上近似的解；此时必须使用 `AMBIGUOUS` 状态、工装先验、CAD ROI 或
位姿范围约束防止误判。

不能只凭 `converged=yes` 接受结果。生产判定至少同时检查状态码、RMSE、内点
比例、最佳与第二候选分差，以及矩阵是否在工位允许范围内。

## 2. 配准流程

```text
输入并过滤有限 XYZ
→ 基础体素降采样
→ INITIAL / PCA / FPFH-RANSAC 生成粗配种子
→ 粗体素快速近邻评分
→ 每种策略只保留最优少量候选
→ 多级 ICP 精配
→ RMSE、内点率和可选目标覆盖率评分
→ 候选去重、排序和歧义判断
→ 输出 source → target 变换
```

`cadreg_set_target()` 会复制并缓存稳定的 CAD 目标。相同句柄后续处理多件工件
时，可复用目标降采样点云和目标 FPFH 特征。句柄不是线程安全的；每个工作线程
使用独立句柄。

## 3. 模式与策略

### 3.1 `mode`

| 值 | 行为 | 建议用途 |
|---|---|---|
| `SINGLE` | 只运行掩码中顺序最靠前的一种策略 | 已确认在线策略 |
| `CASCADE` | INITIAL→PCA→FPFH，某策略通过质量门即停止 | 有分级回退需求 |
| `ENSEMBLE` | 执行所有启用策略并统一排名 | 离线验证、歧义检测 |

`SINGLE + strategy_mask=all` 只会运行 INITIAL。在线使用时应明确指定一种策略。

### 3.2 `strategy_mask`

| 策略 | 数值 | 特点 |
|---|---:|---|
| `INITIAL` | 1 | 从可选初始矩阵开始，最快；ICP仍会修正每件工件偏差 |
| `PCA` | 2 | 主轴粗配较快；局部、对称点云可能发生轴翻转 |
| `FPFH_RANSAC` | 4 | 可无初值全局定位；速度较慢且重复结构可能歧义 |

策略可以按位或组合，例如 `INITIAL | PCA | FPFH_RANSAC`。

## 4. 默认参数

调用 `cadreg_default_options()` 得到：

| 参数 | 默认值 |
|---|---:|
| `max_correspondence_distance` | 5.0 |
| `max_iterations` | 100 |
| `voxel_size` | 1.0 |
| `mode` | `CASCADE` |
| `strategy_mask` | INITIAL、PCA、FPFH |
| `feature_voxel_size` | 5.0 |
| `ransac_max_iterations` | 50000 |
| `min_inlier_ratio` | 0.60 |
| `max_rmse` | 2.0 |
| `ambiguity_score_margin` | 0.03 |
| `icp_level_count` | 3 |
| ICP 粗级 | voxel=5.0，distance=30.0，iterations=40 |
| ICP 中级 | voxel=2.0，distance=10.0，iterations=60 |
| ICP 精级 | voxel=0.8，distance=2.5，iterations=80 |
| `ransac_attempts` | 4 |
| `max_candidates_per_strategy` | 4 |
| `max_refined_candidates_per_strategy` | 2 |
| `enable_target_coverage` | 0 |
| `min_target_coverage` | 0.20 |

距离和体素使用与点云相同的单位。本项目约定为毫米。

## 5. 全部可调参数

### 5.1 单级兼容参数

`max_correspondence_distance` 是质量计算的内点距离；当
`icp_level_count=0` 时也作为单级 ICP 对应距离。增大可容忍更差初值，但增加
错误对应风险。

`max_iterations` 在 `icp_level_count=0` 时控制单级 ICP 最大迭代次数。它是计算
上限，不是固定执行次数。达到收敛条件会提前停止；达到上限后仍必须通过 DLL
质量门槛。

`voxel_size` 是基础降采样尺寸；`<=0` 表示关闭。当 `icp_level_count=0` 时也是
单级 ICP 体素。增大通常显著提速，但会降低小特征和最终位姿分辨率。

当 `icp_level_count>0` 时，真正控制每级 ICP 的是 `icp_levels[]`。测试程序会
根据命令行的三个兼容参数重新生成三级配置，因此测试程序行为与直接调用默认
参数略有不同。

### 5.2 多级 ICP

`icp_level_count` 范围为 0～4。0 使用单级兼容参数；1～4 使用对应数量的
`icp_levels`。

每个 `CadRegIcpLevel` 包含：

- `voxel_size`：该级点云体素；粗级应大于精级。
- `max_correspondence_distance`：该级最大对应距离；应逐级收紧。
- `max_iterations`：该级最大迭代次数；粗级通常少、精级通常多。
- `reserved`：必须设为0。

建议三级关系：

```text
粗：大体素、大距离、15～30次
中：中体素、中距离、20～40次
精：目标体素、小距离、30～60次
```

### 5.3 FPFH 与 RANSAC

`feature_voxel_size` 控制 FPFH 点数。较大更快，较小保留更多局部细节。一般
工业零件可从 3～5 mm 开始；它只影响粗配，不直接限定最终 ICP 精度。

`ransac_max_iterations` 控制一次 RANSAC 的搜索上限。增加可提高困难数据找到
假设的概率，也会增加粗配时间；重复结构歧义不能仅靠增加迭代解决。

`ransac_attempts` 是独立 RANSAC 次数。1最快，2适合快速在线，4更稳健。

`max_candidates_per_strategy` 是每种策略最多保留的粗种子数量。

`max_refined_candidates_per_strategy` 是快速粗评分后，最多进入完整多级 ICP 的
数量，必须不大于 `max_candidates_per_strategy`。这是主要速度开关：

- 1：最快，但同策略没有第二候选保护。
- 2：当前默认，兼顾速度和可靠性。
- 3～4：更稳健，但精配耗时近似按候选数增长。

### 5.4 质量与歧义

`min_inlier_ratio` 范围 0～1。源扫描中距离 CAD 不超过
`max_correspondence_distance` 的点所占比例必须达到此值。

`max_rmse` 是结果验收和评分门槛，不是 ICP 的收敛目标。减小只会使验收更严格，
不会自动提升算法精度。

`ambiguity_score_margin` 用于 ENSEMBLE。最佳和第二个已接受候选的得分差小于此值
时返回 `AMBIGUOUS`。减小该值会减少歧义报警，同时提高误配被当作成功的风险。

`enable_target_coverage` 开启反向目标覆盖率，会增加一次近邻搜索。完整扫描对
完整 CAD 时有意义；局部扫描对完整 CAD 默认关闭。

`min_target_coverage` 仅在目标覆盖率开启时生效。

### 5.5 结构体维护字段

`struct_size` 必须等于当前结构体大小；`abi_version` 必须等于5。所有 `reserved`
字段必须设为0。不要手写默认结构体，先调用 `cadreg_default_options()`，再修改
需要的字段。

## 6. 参数对速度和精度的影响

速度影响通常从大到小为：

1. 进入完整 ICP 的候选数量；
2. ICP 和基础体素尺寸；
3. 每级 ICP 最大迭代次数；
4. FPFH 体素和 RANSAC 尝试次数；
5. RANSAC 最大迭代次数；
6. 是否计算目标覆盖率。

最终精度主要受以下因素影响：

1. 精级 ICP 体素和对应距离；
2. 扫描标定误差、噪声、飞点和背景；
3. 粗配是否进入正确吸引域；
4. 扫描区域是否包含唯一几何；
5. 精级迭代是否充分。

`min_inlier_ratio`、`max_rmse`、`ambiguity_score_margin` 和
`min_target_coverage` 主要控制验收，不直接提高精度。

## 7. 当前项目推荐配置

真实局部扫描的快速 FPFH 配置：

```text
mode                 = SINGLE
strategy             = FPFH_RANSAC
max distance         = 5 mm
final voxel          = 1 mm
final iterations     = 40
feature voxel        = 5 mm
generated seeds      = 4
refined candidates   = 2
```

当前样本首帧实测 DLL 内部约 2.2秒；生产中复用同一目标句柄通常更快。这个数字
不包含 PLY 文件读取和写出。生产程序应直接传递内存点云，不要每件工件落盘后
再读取。

如果有工装名义矩阵，推荐 `SINGLE + INITIAL`。名义矩阵只提供搜索起点，每件
工件的平移和旋转偏差仍由多级 ICP 修正，因此不会固定最终结果。

## 8. 状态码和生产判定

| 状态 | 含义 | 是否可直接使用 |
|---|---|---|
| `SUCCESS` | 最佳候选通过质量和歧义判定 | 仍需检查工位位姿范围 |
| `NOT_CONVERGED` | 没有可靠收敛 | 否 |
| `INVALID_ARGUMENT` | ABI或参数错误 | 否 |
| `EMPTY_AFTER_DOWNSAMPLING` | 降采样后点数不足 | 否 |
| `INTERNAL_ERROR` | 内部异常 | 否 |
| `QUALITY_REJECTED` | 收敛但质量门未通过 | 否 |
| `AMBIGUOUS` | 多个候选无法区分 | 否，需先验或人工确认 |

`CadRegResult` 还返回矩阵、RMSE、内点率、得分、第二得分、候选数量、阶段耗时、
缓存命中以及最多16条候选诊断。失败状态下矩阵仅用于诊断，不能驱动生产设备。

## 9. 工业部署要求

上线前至少使用多批合格件、缺陷件和极限位置样件统计：

- 正确配准率和误配率；
- RMSE、内点率和分差分布；
- 平移、旋转误差分布；
- P50、P95、P99和最大耗时；
- 重复扫描的位姿重复性；
- 缺点、遮挡、反光和背景残留情况下的失败行为。

建议根据统计分布设置门槛，而不是只用单帧结果。固定型号还应增加业务层位姿
范围检查，例如允许的 XYZ 偏移和各轴旋转范围。

## 10. 已知边界与后续优化

当前版本已经具备纯 C ABI、目标缓存、多策略、候选预筛、多级 ICP、质量门和
歧义状态，适合当前受控工位继续验证。仍可继续增加：

- point-to-plane ICP 或 GICP，提高曲面收敛速度与精度；
- 平移/旋转搜索范围及 CAD ROI 约束；
- 可配置随机种子，保证 FPFH 回归可复现；
- 单次调用硬超时和取消机制；
- 鲁棒核、法向一致性和边界点抑制；
- 输入点数上限或自适应体素；
- 连续工件的名义位姿和上一帧位姿融合。

这些功能会提高通用性和可控性，但不能替代现场数据验收。

## 11. 当前内部性能优化

- 每级 CAD ICP 点云对应的 KD 树保存在长生命周期句柄中，多个候选和连续工件
  复用，不再为每个候选重复建立目标搜索结构。
- 某一级体素与基础体素相同时直接复用基础点云，避免重复执行相同降采样。
- 目标 FPFH 点云直接由目标原始副本按特征体素生成；只改变 ICP 基础体素时，
  相同 `feature_voxel_size` 的目标特征缓存不会失效。
- 粗候选仍先进行低成本评分，只有 `max_refined_candidates_per_strategy` 指定的
  最优候选进入完整多级ICP。

当前 `sample_bottom.ply` 推荐配置的回归结果从约3.62秒降至约2.64秒，RMSE由
约0.94058 mm保持为约0.94054 mm，内点率保持1.0。不同运行的CPU调度会导致
耗时波动，生产验收应统计P95和P99，而不是只看单次数据。
