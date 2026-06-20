# AlgInterface Deep Dive Report

生成时间：2026-06-20

## 结论

1. 不能通过当前新版 `USERSDK.dll` 的公开导出开关恢复原来的 `640x480` 眼图回调。已测试 `_7_CALL_set_eye_image_state`、`_7_CALL_switch_camera_image_process`、`_7_CALL_set_run_mode`、`_7_CALL_set_camera_fps`、不同 `_7_CALL_init_sdk` 参数，结果仍固定为 `80x60/4800`。

2. 眼图分辨率由 `USERSDK.dll` 决定，不是由 `AlgInterface.dll` 决定。矩阵测试显示：
   - 新 `USERSDK.dll`：`80x60/4800`
   - 旧 `USERSDK.dll`：`640x480/307200`
   - 旧 `USERSDK.dll` + 当前 `AlgInterface.dll`/support DLL：仍能进入相机、gaze callback 和 `_7_CALL_start_tracking` 路径，并返回 `640x480/307200` 眼图。

3. 唯一满足“高分辨率眼图和 gaze/pupil 使用同一 SDK 设备时间戳”的候选路径是：
   - `USERSDK.dll` 使用替换前备份的旧版；
   - `AlgInterface.dll`、`CertPlatform.dll`、`SdkEvent.dll`、`smoothAbout.dll`、`sqlite3.dll`、`libeay32.dll`、`zlib1.dll` 使用当前 `C:\Third_Party\AlgInterface` 版本；
   - 仍通过 `_7_CALL_start_tracking(..., split1024-sized coefficient)` 输入 Runtime 公共 API 读出的 2048 字节校准参数。

4. OpenCV/DirectShow 旁路可以读到 `640x480`，但不能作为最终方案：它没有 SDK 的 device timestamp。尝试用新版 SDK 的 `80x60` 帧作为时间戳锚进行内容匹配时，持续采集不稳定，且已得到的匹配相关性接近 0。这个旁路最多能做调试预览，不能满足“时间戳务必不能出错”。

5. 当前机器状态下，低层 USERSDK 已进入“`_7_CALL_start_camera -> 0` 但 callback 为 0”的阻塞状态。该状态同时影响旧版和新版 `USERSDK.dll`，所以不是 AlgInterface 文件差异造成的。Runtime 公共 API 仍能连接并读取 coefficient，OpenCV 也仍能读高分辨率相机，说明硬件没有离线；更像是 VR/Tobii/VIVE 运行时占用或设备接口状态没有释放。`Tobii VRU02 Runtime` 服务当前以 LocalSystem 运行，普通权限无法停止。

## 关键证据

### 文件和二进制差异

- 旧 `AlgInterface.dll`：约 `33,036,800` 字节。
- 当前 `AlgInterface.dll`：`469,504` 字节。
- 两版 `USERSDK.dll` 导出函数名一致，`AlgInterface.dll` 导出也一致。
- 旧 `AlgInterface.dll` 字符串包含 OpenCV/dnn/objdetect、`Quick`、`SlowOne`、`SlowTwo`、`eyeValid.xml`、`GlassRTrees.xml` 等资源痕迹。
- 当前 `AlgInterface.dll` 字符串包含 `PupilGlintDetector.cpp`、`PupilDiameter`、`ImageCoefficient`、`ImageOffsetX/Y`、`GlintMinWidth/MaxWidth` 等，算法实现明显重写。

差异分析输出目录：

`C:\Third_Party\PythonEyetracker\PythonScript\alginterface_diff_20260620_113300`

### 组合矩阵

矩阵测试输出：

`C:\Third_Party\PythonEyetracker\PythonScript\alginterface_matrix_20260620_113535\matrix_summary.csv`

关键行：

- `new_all`：`80x60/4800`
- `old_all`：`640x480/307200`
- `old_usersdk_new_alg_new_support`：`640x480/307200`，gaze callback 有 rows，进程不崩溃。

### 当前阻塞状态

最近健康检查结果：

- `C:\Third_Party\PythonEyetracker\PythonScript\usersdk_after_stop_services_test`
- `C:\Third_Party\PythonEyetracker\PythonScript\hires_current_alg_sync_test_20260620_121118`

现象：

- `_7_CALL_init_sdk -> 0`
- `_7_CALL_set_camera_image_callback -> 0`
- `_7_CALL_start_camera(17178, 17156) -> 0`
- `_7_CALL_start_eye_gaze_callback_ex -> 0`
- `_7_CALL_start_tracking(...) -> 0`
- `images=0`，`gaze=0`

同时：

- Runtime 公共 API `aSeeVR_get_coefficient -> 0`，能拿到 2048 字节 coefficient。
- OpenCV/DirectShow 能读 `640x480`。
- `EyeCalibrationDashboard.exe`、SteamVR/VIVE 相关进程曾在运行。
- `Tobii VRU02 Runtime` 服务 PID 5244 当前无法用普通权限停止：`Access is denied`。

## 新一键脚本

脚本：

`C:\Third_Party\PythonEyetracker\PythonScript\capture_hires_current_alg_sync_test.ps1`

建议佩戴并完成校准后运行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File C:\Third_Party\PythonEyetracker\PythonScript\capture_hires_current_alg_sync_test.ps1 -Timeout 8 -MaxImages 2500 -MaxGaze 2500 -RequireValidSignals
```

脚本行为：

- 备份当前 runtime 8 个 DLL 到 `C:\7invensun\aSeeVR_UserSDK\runtime\backup_before_hires_current_alg_*`。
- 刷新 coefficient。
- 尽量停止 Runtime、SteamVR/VIVE/SRanipal 相关占用进程。
- 临时安装“旧 `USERSDK.dll` + 当前 `AlgInterface`/support DLL”组合。
- 调用现有同步采集脚本，输出 eye image、gaze、pupil CSV 和高分辨率 PGM 帧。
- 计算 image/gaze 采样率、有效 gaze/pupil 数、timestamp 命中数。
- 调用 `visualize_usersdk_gaze.py` 生成轨迹图。
- 默认恢复原 runtime 文件；如需保留组合，可加 `-KeepRuntimeCombo`。

通过标准：

- `image sizes` 包含 `640x480/307200`。
- `gaze rows > 0`。
- 佩戴且校准正常时，`valid recommended gaze`、`valid left pupil`、`valid right pupil` 应明显大于 0。
- `image stereo timestamp Hz` 应接近旧高分辨率路径的双眼同步帧率；`gaze timestamp Hz` 应接近旧路径的 gaze callback 频率。

## 建议下一步

当前阻塞不是代码路径，而是设备占用/权限状态。建议在运行一键脚本前先做其中之一：

- 关闭 VIVE/SteamVR/SRanipal/EyeCalibrationDashboard。
- 以管理员权限停止 `Tobii VRU02 Runtime`，或重启机器后不要启动 Tobii/VIVE/SteamVR，再运行脚本。
- 若仍为 `images=0/gaze=0`，需要管理员权限执行 `pnputil /restart-device` 重置 `Droolon F1` 设备，或物理重新插拔眼动仪/头显 USB。

在没有恢复低层 USERSDK callback 前，无法诚实地确认“稳定复现”；但根据矩阵测试，候选路径已经明确，且是目前唯一保留 SDK device timestamp 的路径。
