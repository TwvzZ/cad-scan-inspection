# Registration DLL

源码位于当前目录，构建输出位于 `../../build/registration/Release`。

默认测试：

```powershell
./run_registration_test.ps1
```

默认读取 `../../data/input/scan/zhujian_1seg.ply` 和 `../../data/output/sampling/vg1500040104a_sampled.ply`，输出到 `../../data/output/registration/`。

支持 `single`、`cascade`、`ensemble` 模式，以及 `initial`、`pca`、`fpfh`、`all` 策略。
