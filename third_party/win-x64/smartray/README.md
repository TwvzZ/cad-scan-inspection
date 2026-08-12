# SmartRay x64 runtime

该目录来自本机 SmartRay DevKit 的 `SR_API/Win_x64_MSVC_19/bin`，供工控机免安装部署。

- `SRAPI_CSharp-x64.dll`：C# API。
- `SR_API-x64.dll`：SmartRay 原生 API。
- `PocoFoundation64.dll`、`glib/gio/gobject/gmodule/gthread`：SDK 自带依赖。
- `msvcp100.dll`、`msvcr100.dll`：SDK 自带旧组件所需的 Microsoft VC++ 2010 SP1 x64 Runtime。
- `params/ECCO95_3D_Snapshot.par`：ECCO95 官方 3D 单次点云参数集，连接时加载一次。
- `params/ECCO95_3D_Repeat_Snapshot.par`：ECCO95 官方连续点云参数集，用于高频连续采集。

当前适配器仅接受 ECCO95 型号；不得将该参数集用于 ECCO95 Plus 或其他 SmartRay 型号。

更新 SmartRay DevKit 后应重新核对文件哈希、DLL 依赖和设备驱动兼容性。
