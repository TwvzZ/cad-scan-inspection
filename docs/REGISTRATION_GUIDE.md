# 配准、配置与调试指南

本文面向需要修改参数、分析配准质量或维护算法的用户。首次安装、六样本测试和真实点云运行命令请先看项目根目录的 [README](../README.md)。

## 1. 数据流与输入边界

实际项目提供三类输入：

1. STEP 模型：`cad.step_path`；
2. 检测面区域：`detection_faces`；
3. 3D 扫描点云：`scan.path` 或命令行 `--scan`。

CAD 参考点云不是外部输入。首次克隆时它不存在，由 `cadscan-sample-step` 从 STEP 生成到 `cad.point_cloud_output_path`。

```text
STEP ──采样──> CAD reference PLY ──┐
                                    ├─> 粗配准 ─> ICP ─> ROI ─> 平面度
3D 扫描点云 ────────────────────────┘
```

## 2. 唯一配置文件

全部参数集中在 `configs/project.json`，相对路径均相对于该 JSON 所在目录解析。

### `cad`

- `step_path`：项目提供的 STEP/STP 文件；
- `point_cloud_output_path`：待生成 CAD 点云的目标位置；
- `face_catalog_path`：STEP 全部面的分类与核查结果输出位置；
- `sampling.point_count`：参考点数量；
- `sampling.method`：`uniform` 或 `poisson`；
- `sampling.unit`：STEP 目标单位，通常为 `MM`；
- `linear_tolerance`、`angular_tolerance`：STEP 三角化精度。

采样点越多，几何覆盖越细，但内存、磁盘和配准时间都会增加。STEP 或采样参数未改变时应复用已有参考点云。

### `scan`

- `path`：默认扫描点云；
- `max_abs_coordinate`：坐标绝对值上限，超过该值的点会作为异常点移除。

命令行 `--scan` 会覆盖 `scan.path`，适合设备连续产生新文件的场景。

### `initial_pose`

- `mode`：`pca_robust`、`fpfh_ransac`、`fixture` 或 `manual`；
- `voxel_size`：粗配准降采样体素；
- `distance_factor`：RANSAC 对应距离与体素的倍数；
- `estimate_scale`：是否估计统一尺度；
- `attempts`：独立粗配准尝试次数；
- `max_iterations`、`confidence`：RANSAC 终止条件；
- `matrix`：工装或人工初始矩阵。

`pca_robust` 是默认模式，适合当前长条形、重复结构且扫描只覆盖部分表面的工件。它生成四个合法 PCA 方向候选，以裁剪最近邻迭代估计统一尺度和姿态，再用紧距离平移搜索抑制毛刺或块状缺陷造成的法向偏移。`pca_*` 配置项分别控制迭代次数、初始/最小对应距离、裁剪比例、评分距离和平移搜索范围。

真实设备坐标单位一致时应使用 `estimate_scale: false`。固定仿真样本含人为尺度变化，所以 README 的测试命令显式传入 `--estimate-scale`。

### `icp`

- `voxel_size`：精配准降采样体素；
- `max_correspondence_distance`：最大对应距离；
- `method`：`point_to_plane` 或 `point_to_point`；
- `max_iterations`：最大迭代次数；
- `relative_fitness`、`relative_rmse`：收敛条件。

体素和对应距离必须与点间距及模型单位匹配。对应距离过小可能无法收敛，过大会接受错误对应。

### `roi`

- `height_range`：沿检测面法向保留的范围；
- `xy_margin`：检测面边缘余量；
- `plane_ransac.distance_threshold`：平面内点距离；
- `ransac_n`、`num_iterations`、`probability`：平面 RANSAC 参数。

### `detection_faces`

每个检测面使用 STEP 全局模型坐标：

```json
{
  "name": "face_01",
  "step_face_id": 187,
  "center": [-23.036, -20.449, -39.95],
  "size": [67.686, 54.568],
  "normal": [0, 0, -1],
  "x_direction": [0, 1, 0]
}
```

- `center`：检测面中心；
- `size`：面内两个方向的长度；
- `normal`：有方向的法向；
- `x_direction`：面内局部 X 方向；
- `step_face_id`：辅助追溯原始 STEP 面。

可用以下命令解析 STEP 全部面。输出 JSON 包含总面数、几何类型统计、按类型分类的完整面记录，以及按筛选条件得到的平面检测候选。分类名称取自 CadQuery/OpenCascade 并动态创建；`known_types` 是当前库版本的已知类型，`observed_types` 仅表示当前模型实际存在的类型，不限制其他 STEP 模型：

```powershell
cadscan-inspect-cad configs/project.json
```

核查输出不替代配置；生产检测区域始终以 `detection_faces` 为准。

### `output_group`、`output_name` 与 `output_dirs`

- `output_group`：未指定命令行参数时使用的结果类别，取值为 `test` 或 `actual`；
- `output_name`：未指定命令行参数时使用的结果次级目录名称；
- `output_dirs.test`：测试样本结果根目录；
- `output_dirs.actual`：真实点云结果根目录。

程序使用 `output_name` 在对应根目录下建立次级目录，不依赖点云文件名。命令行 `--output-group` 可切换类别，`--output-name` 可指定工件号或批次号；`--output-dir` 仅用于需要完全自定义单次输出位置的场景。

## 3. 坐标与变换约定

最终矩阵把 CAD 模型坐标映射到扫描坐标：

```text
p_scan = S * R * p_cad + T
```

- `S`：统一尺度；
- `R`：3×3 旋转；
- `T`：平移；
- `cad_to_scan_matrix.txt`：对应的齐次 4×4 矩阵。

真实刚体配准中 `S = 1`。ICP 使用粗配准结果作为初值，并优化后续刚体修正。

## 4. 配准阶段

### 鲁棒 PCA

默认先根据 CAD 与扫描点云的主轴和有向包围盒生成姿态候选，再把局部扫描匹配到完整 CAD。候选评分以扫描覆盖率为准，不会因为 CAD 中存在扫描未覆盖的区域而降低正确结果分数。紧距离平移搜索用于避免大缺陷把整体姿态拉向相邻平面。

### FPFH + RANSAC

对 CAD 与扫描点云降采样、估计法向并计算 FPFH 特征，再通过 RANSAC 获得全局初始位姿。它解决初始位置差异较大的问题，但精度通常不足以直接检测。

该模式保留为非规则、非重复模型的备选方案。对于当前重复结构和局部表面扫描，优先使用 `pca_robust`。

重点检查：

- 粗配准 `fitness` 是否过低；
- `inlier_rmse` 是否明显大于点间距；
- 点云单位是否一致；
- 扫描是否保留足够几何特征。

### ICP

在粗配准基础上进行局部精配准。平面类零件通常使用 `point_to_plane`；若法向质量较差，可尝试 `point_to_point`。

重点检查：

- ICP fitness 是否比粗配准提高；
- RMSE 是否下降；
- 对齐点云是否落在正确结构上；
- 是否因对称结构落入错误位置。

## 5. ROI 与平面度

程序将配置中的检测面通过最终矩阵映射到扫描坐标系，再按面内尺寸、边缘余量和法向高度截取 ROI。

每个 ROI 继续执行平面 RANSAC，并计算：

- 峰谷平面度；
- 稳健 P95-P05 平面度；
- RANSAC 内点峰谷值；
- 内点 RMSE；
- 内点率；
- 拟合法向与目标法向的角度误差。

ROI 点数为零时，优先检查检测面坐标、法向、最终矩阵和 `height_range`。

## 6. 输出说明

- `cad_aligned.ply`：对齐到扫描坐标的 CAD 点云；
- `cad_to_scan_matrix.txt`：最终 4×4 矩阵；
- `registration_result.json`：输入、矩阵、尺度、配准指标和 ROI 记录；
- `roi_*_red.ply`：各检测区域；
- `plane_*_blue.ply`：平面 RANSAC 内点；
- `roi_all_red.ply`：全部 ROI；
- `fitted_planes_all_blue.ply`：全部拟合平面；
- `flatness_summary.csv`：结构化检测结果；
- `inspection_report.txt`：人工阅读报告。

`registration_result.json` 中的输入文件使用相对路径记录，工程移动后仍可追溯。

每次运行还会在 `registration_result.json` 的 `timing_seconds` 中记录点云读取、粗配准、ICP、对齐 CAD 输出、ROI/平面拟合及处理总耗时。每个 ROI 的 `processing_seconds` 记录该面的提取、拟合和文件输出耗时；相同汇总也会写入 `inspection_report.txt` 并打印到终端。所有时间均为单调高精度时钟测得的秒数，`processing_total` 截止到主要点云结果生成完成，不包含最后几个文本结果文件自身的写入时间。

## 7. 调试顺序

出现异常时建议依次检查：

1. STEP、扫描点云和配置单位是否一致；
2. CAD 参考点云是否由当前 STEP 和采样参数生成；
3. 输入点云是否为空、损坏或包含异常坐标；
4. 粗配准 fitness、RMSE 和变换方向；
5. ICP 是否改善粗配准；
6. 对齐后的 `cad_aligned.ply` 是否正确；
7. 检测面中心、尺寸、法向与 `height_range`；
8. ROI 点数、平面内点率和法向角误差。

环境安装、标准运行命令和六样本批量命令不在本文重复，请回到 [README](../README.md)。
