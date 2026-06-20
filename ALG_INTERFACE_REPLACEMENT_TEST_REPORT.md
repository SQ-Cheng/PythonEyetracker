# AlgInterface Replacement Test Report

Date: 2026-06-20

## Replacement

Source directory:

`C:\Third_Party\AlgInterface`

Runtime directory:

`C:\7invensun\aSeeVR_UserSDK\runtime`

Backed up original runtime files to:

`C:\7invensun\aSeeVR_UserSDK\runtime\backup_before_alginterface_20260620_111308`

The following files were replaced and verified by SHA256 hash against the source files:

- `AlgInterface.dll`
- `CertPlatform.dll`
- `libeay32.dll`
- `SdkEvent.dll`
- `smoothAbout.dll`
- `sqlite3.dll`
- `USERSDK.dll`
- `zlib1.dll`

Backup manifests:

- `backup_manifest.csv`
- `replacement_verify_manifest.csv`

## Verification Runs

### Coefficient via Runtime public API

After replacement, Runtime public API can still return the 2048-byte coefficient:

`C:\Third_Party\PythonEyetracker\PythonScript\coefficient_after_replace_probe.bin`

Result:

- `aSeeVR_connect_server -> 0`
- `aSeeVR_get_coefficient -> 0`
- `coefficient callback: received 2048 bytes`

### USERSDK synchronized capture

Test command:

`powershell.exe -NoProfile -ExecutionPolicy Bypass -File C:\Third_Party\PythonEyetracker\PythonScript\capture_usersdk_sync_test.ps1 -Timeout 10 -MaxImages 80 -MaxGaze 2000`

Latest output directory:

`C:\Third_Party\PythonEyetracker\PythonScript\usersdk_sync_test_20260620_111409`

Result:

- `_7_CALL_init_sdk -> 0`
- `_7_CALL_set_camera_image_callback -> 0`
- `_7_CALL_start_camera(17178, 17156) -> 0`
- `_7_CALL_start_eye_gaze_callback_ex -> 0`
- `_7_CALL_start_tracking(1, split1024-sized[0]) -> 0`
- `_7_CALL_start_tracking(2, split1024-sized[1]) -> 0`
- `_7_CALL_stop_tracking -> 0`
- `_7_CALL_stop_eye_callback -> 0`
- `_7_CALL_stop_camera -> 0`
- `_7_CALL_release -> 0`

Captured data:

- Image callbacks: 4530
- Saved image rows: 80
- Gaze callback rows: 2001
- Image dropped: 0
- Gaze dropped: 0

Important difference from the previous working runtime:

- Eye image frame size is now `80x60`, `4800` bytes per eye frame.
- Previous runtime returned `640x480`, `307200` bytes per eye frame.

Decoded gaze/pupil result in this run:

- Valid recommended gaze rows: 0
- Valid left gaze rows: 0
- Valid right gaze rows: 0
- Valid left pupil rows: 0
- Valid right pupil rows: 0

However, some non-gaze status fields changed over time, for example `left_openness` and `right_openness`, which suggests the callback structure is still at least partially compatible and the callback is not simply dead.

## Current Conclusion

After replacing the runtime files with `C:\Third_Party\AlgInterface`, the previous path is only partially open:

- Runtime public coefficient path still works.
- Direct USERSDK camera image callback still works.
- Direct USERSDK gaze callback still fires.
- Direct `_7_CALL_start_tracking` still accepts the known `split1024-sized` coefficient layout.
- Synchronized timestamps are still present for image and gaze callback rows.

But the old "raw image + calibrated gaze/pupil" route is not fully restored under this replacement because the decoded gaze and pupil fields remain zero in the verification run.

The replacement also changes the image stream from full `640x480` frames to `80x60` frames, so this AlgInterface package appears to expose a different/processed eye image stream than the previous runtime package.

## Rollback

To restore the original runtime files, stop Runtime first, then copy the 8 backed-up files from:

`C:\7invensun\aSeeVR_UserSDK\runtime\backup_before_alginterface_20260620_111308`

back into:

`C:\7invensun\aSeeVR_UserSDK\runtime`

