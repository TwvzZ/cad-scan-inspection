# CAD Scan Inspection

本项目当前为 C++ DLL 工程，包含 STEP 采样、点云配准和检测三个模块。Python 旧实现已移除。

数据目录约定：

- `assets/cad`：原始 STEP，只读保留。
- `data/input/scan`：原始扫描点云。
- `data/output/sampling`：采样生成的 CAD 点云。
- `data/output/registration`：配准后的点云。
- `data/output/inspection`：检测点云、日志和报告。

三个模块可按“采样 → 配准 → 检测”直接串联，不需要复制中间文件。详细构建和测试命令见 `CPP开发说明.md`。
