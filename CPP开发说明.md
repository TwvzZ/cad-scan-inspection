# CAD Scan Inspection C++ 工程

## 目录

```text
assets/cad/                         原始 STEP（保留，不复制）
assets/face_catalogs/               CAD 面信息
native/registration|sampling|inspection  三个 DLL 源码和测试源码
build/registration|sampling|inspection  CMake 构建输出
 data/input/scan/                   原始扫描点云
 data/output/sampling/              CAD 采样点云
 data/output/registration/          配准点云
 data/output/inspection/            检测结果与日志
tests/                              仅保留测试源码，不存运行数据
```

## 数据流

```text
assets/cad/*.stp
  -> data/output/sampling/*.ply
  -> data/output/registration/*_aligned.ply
  -> data/output/inspection/*
```

## 构建

```powershell
cmake --build build/registration --config Release
cmake --build build/sampling --config Release
cmake --build build/inspection --config Release
```

## 默认测试

```powershell
native/sampling/run_sampling_test.ps1
native/registration/run_registration_test.ps1
build/inspection/Release/cadinspect_ply_test.exe `
  data/output/sampling/vg1500040104a_sampled.ply `
  data/output/registration/zhujian_1seg_aligned.ply `
  data/output/inspection/smoke
```

采样脚本默认读取 `assets/cad/vg1500040104a.stp` 并写入采样输出；配准脚本默认读取原始扫描和采样输出，并写入配准输出。
