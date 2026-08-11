# 点云 ROI、平面、平面度与缺陷检测 DLL

`cad_inspection.dll` 是独立的纯 C ABI 模块，不依赖或暴露 PCL、Eigen、OpenCASCADE
和 STL 类型。输入为内存中的交错 `double XYZ`，不包含文件读取。

当前接口版本为 `CADINSPECT_ABI_VERSION = 3`。

## 处理逻辑

```text
CAD局部坐标框架
→ 高ROI提取完整检测面
→ CAD名义平面附近窄ROI提取基准候选
→ 法向约束RANSAC
→ RANSAC内点最小二乘精拟合
→ 基准点数量、覆盖率、RMSE和法向可信度验收
→ 完整高ROI独立计算平面度
→ 所有点相对参考平面计算有符号距离
→ 正/负缺陷阈值分割和二维连通聚类
```

窄 ROI 只用于建立缺陷高度参考，不参与完整面平面度的合格判定。即使块状凸起
面积大于正常面，只要它位于窄 ROI 高度范围之外，就不会被拟合成基准面。

## 输入

### 点云

- `xyz`：调用方持有的交错双精度 XYZ。
- `point_count`：点数。
- `xyz_stride_bytes`：点跨度；0表示三个连续 `double`。

DLL只在调用期间借用输入指针，不保留、不释放调用方内存。

### CAD局部框架

- `origin`：CAD名义检测面上的原点。
- `axis_u`、`axis_v`：检测面的两个面内方向。
- `nominal_normal`：从正常面指向正缺陷的法向。

DLL内部会归一化和正交化这些轴。所有点先转换为局部 `(u,v,w)`，ROI因此能够
随CAD检测面姿态变化，不依赖世界坐标轴。

### ROI参数

- `u_min/u_max/v_min/v_max`：检测面的横向边界。
- `detection_normal_min/max`：高ROI，必须覆盖正常面和允许检测的最高/最低缺陷。
- `reference_normal_min/max`：窄ROI，只用于正常基准候选。
- `edge_margin`：从横向边缘向内排除的距离，避免侧壁、圆角和边缘噪声进入拟合。
- `reference_mode`：`NARROW_ROI` 使用窄高度层；`WIDE_MULTIPLANE` 在宽ROI中
  依次拟合并剥离多个平面。
- `max_plane_candidates`：宽ROI最多提取的平面数，默认4、最大8。
- `min_candidate_point_ratio`：宽ROI候选相对全部参考点的最小点数比例。

窄ROI宽度必须大于配准法向残差和扫描噪声之和，同时小于需要排除的最小缺陷
高度。如果真实数据完全没有正常面点，模块返回 `INSUFFICIENT_REFERENCE` 或
`PLANE_REJECTED`，不会将块状缺陷顶面强制当作基准面。

### 平面参数

- `ransac_max_iterations`：RANSAC上限。
- `random_seed`：固定回归结果。
- `ransac_evaluation_limit`：每轮假设评分的代表点上限，默认5000；最终内点和
  质量仍使用完整窄ROI。
- `ransac_confidence`：根据当前最佳内点率自适应提前结束的置信度，默认0.999。
- `plane_inlier_distance`：参考平面内点距离。
- `max_normal_angle_deg`：拟合法向相对CAD法向的最大角度。
- `min_reference_points`：最少参考点数。
- `min_reference_inlier_ratio`：最小参考内点比例。
- `coverage_grid_u/v`：空间覆盖率网格。
- `min_reference_grid_coverage`：最小被占用网格比例。
- `max_reference_rmse`：参考内点最大拟合RMSE。

点数多但只集中在一个角落时，覆盖率门槛会拒绝不稳定的平面。

### 缺陷参数

- `positive_defect_threshold`：正向凸起阈值。
- `negative_defect_threshold`：负向凹陷阈值。
- `defect_cluster_cell_size`：面内聚类网格尺寸。
- `min_defect_points`：最小缺陷点数。
- `min_defect_area`：最小投影面积。
- `max_output_defects`：最多返回的缺陷数量。

### 平面度参数

- `flatness_trim_fraction`：稳健诊断值每端裁掉的比例；不影响完整极差。
- `flatness_working_grid_size`：最小区域迭代的UV网格尺寸；每格保留最高和最低
  点，默认0.5。完整最小二乘和稳健统计仍使用全部检测点。
- `minimum_zone_max_iterations`：最小区域近似优化上限。
- `minimum_zone_tolerance`：平面斜率搜索停止阈值。

## 输出

### 参考平面

- 世界坐标平面 `Ax+By+Cz+D=0`。
- 单位法向和中心点。
- 相对CAD法向的角度、相对CAD面的偏移。
- RMSE、平均绝对误差、最大绝对误差。
- 候选点数、内点数、内点比例。
- 网格占用数、覆盖率和 `reliable`。
- 宽ROI模式返回全部候选的平面、偏移、RMSE、点数比例、覆盖率、是否合格及
  最终选中标记。

### 平面度

- `least_squares_peak_to_valley`：完整高ROI对全点最小二乘面的峰谷差，包含缺陷。
- `robust_peak_to_valley`：裁尾后的抗飞点诊断值，不能替代产品总平面度。
- `minimum_zone_flatness`：迭代优化的最小区域近似值。
- 最小区域法向、迭代次数和是否达到数值停止条件。

该模块输出的是点云离散测量结果。正式产品判定必须结合图纸要求、扫描分辨率、
滤波规范和量具相关性试验确定采用哪个平面度字段。

### 每个缺陷

- ID和正/负类型。
- 点数和投影面积。
- 最大、最小和平均有符号高度。
- 估算体积。
- 世界坐标中心。
- 局部U/V/高度包围盒。
- 可选的逐输入点缺陷标签。

### 诊断

- 输入、有限点、高ROI、窄ROI和边缘排除点数。
- 正/负超差点数。
- ROI、平面、平面度、缺陷聚类和总耗时。
- RANSAC实际迭代数、RANSAC评分点数和平面度工作集点数。
- 状态码和错误信息。

## 状态码

- `SUCCESS`：参考面可靠，分析完成。
- `INVALID_ARGUMENT`：ABI、轴、范围或参数错误。
- `EMPTY_DETECTION_ROI`：高ROI点数不足。
- `INSUFFICIENT_REFERENCE`：窄ROI参考点不足。
- `PLANE_REJECTED`：参考平面未通过覆盖率/RMSE/法向等门槛。
- `BUFFER_TOO_SMALL`：缺陷输出容量不足；`required_defect_capacity`返回需求。
- `INTERNAL_ERROR`：内部异常。

## 构建与验证

```powershell
cmake -S cpp_inspection -B build_inspection -A x64
cmake --build build_inspection --config Release
.\build_inspection\Release\cadinspect_test.exe
```

测试程序生成带倾斜、噪声、小毛刺和大面积块状凸起的合成点云，验证窄ROI参考
拟合、完整平面度和两类尺寸缺陷聚类。

## 性能与复杂度

- ROI、最终平面质量、完整面统计：`O(N)`。
- RANSAC：由 `ransac_evaluation_limit` 和自适应置信度控制，不再固定执行
  `最大迭代数 × 全部参考点`。
- 最小区域迭代：使用每个UV网格的高低极值点，复杂度由表面面积和网格尺寸控制，
  不直接随原始点云密度无限增长。
- 缺陷聚类：以带符号的占用网格为节点，每个点和网格只访问常数次，约为
  `O(N+C)`，避免高密度单元中的逐点重复邻域扫描。

20,301点合成回归数据在同一设备上由约29 ms降至约16 ms，平面、平面度和两个
缺陷的结果保持一致。实际耗时应使用目标设备和真实点数统计P95/P99。

## 产物

- `build_inspection/Release/cad_inspection.dll`
- `build_inspection/Release/cad_inspection_wide.dll`
- `build_inspection/Release/cad_inspection.lib`
- `include/cad_inspection.h`
- `examples/CadInspectionNative.cs`
- `build_inspection/Release/cadinspect_test.exe`
