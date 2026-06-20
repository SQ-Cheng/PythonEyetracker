param(
    [double]$Timeout = 8.0,
    [int]$MaxImages = 2500,
    [int]$MaxGaze = 2500,
    [string]$OutputDir = "",
    [string]$CoefficientBin = "",
    [switch]$KeepRuntimeCombo,
    [switch]$NoRestartRuntime,
    [switch]$SkipStopCompetingProcesses,
    [switch]$RequireValidSignals
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RuntimeDir = "C:\7invensun\aSeeVR_UserSDK\runtime"
$RuntimeExe = Join-Path $RuntimeDir "Runtime.exe"
$OldBackupDir = Join-Path $RuntimeDir "backup_before_alginterface_20260620_111308"
$CurrentAlgDir = "C:\Third_Party\AlgInterface"
$Python = "C:\Python311\python.exe"
$CaptureWrapper = Join-Path $ScriptDir "capture_usersdk_sync_test.ps1"
$CoefficientScript = Join-Path $ScriptDir "get_aseevr_coefficient.py"
$VisualizeScript = Join-Path $ScriptDir "visualize_usersdk_gaze.py"

$RuntimeFiles = @(
    "AlgInterface.dll",
    "CertPlatform.dll",
    "libeay32.dll",
    "SdkEvent.dll",
    "smoothAbout.dll",
    "sqlite3.dll",
    "USERSDK.dll",
    "zlib1.dll"
)

function Assert-FileExists([string]$Path) {
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "Required file not found: $Path"
    }
}

function Copy-Checked([string]$Source, [string]$Destination) {
    Assert-FileExists $Source
    Copy-Item -LiteralPath $Source -Destination $Destination -Force
}

function Backup-RuntimeFiles([string]$BackupDir) {
    New-Item -ItemType Directory -Force -Path $BackupDir | Out-Null
    foreach ($name in $RuntimeFiles) {
        Copy-Checked (Join-Path $RuntimeDir $name) (Join-Path $BackupDir $name)
    }
}

function Restore-RuntimeFiles([string]$BackupDir) {
    foreach ($name in $RuntimeFiles) {
        Copy-Checked (Join-Path $BackupDir $name) (Join-Path $RuntimeDir $name)
    }
}

function Install-HiResCurrentAlgCombo() {
    foreach ($name in $RuntimeFiles) {
        $src = if ($name -eq "USERSDK.dll") {
            Join-Path $OldBackupDir $name
        } else {
            Join-Path $CurrentAlgDir $name
        }
        Copy-Checked $src (Join-Path $RuntimeDir $name)
    }
}

function Stop-CompetingProcesses() {
    Write-Host "Stopping Runtime and known VR/VIVE/SRanipal processes that can hold the camera..."
    Get-Process -Name Runtime -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue

    $processNames = @(
        "EyeCalibrationDashboard",
        "UnityCrashHandler64",
        "vrserver",
        "vrcompositor",
        "vrdashboard",
        "vrmonitor",
        "vrwebhelper",
        "steamtours",
        "vivelink",
        "ViveProSettings",
        "ViveDashboard",
        "Vive",
        "ViveportDesktopService",
        "platform_runtime_VR4U2P2_service"
    )
    foreach ($proc in Get-Process -ErrorAction SilentlyContinue) {
        if ($processNames -contains $proc.ProcessName) {
            Write-Host "  stopping $($proc.ProcessName) PID $($proc.Id)"
            try {
                Stop-Process -Id $proc.Id -Force -ErrorAction Stop
            } catch {
                Write-Host "  could not stop $($proc.ProcessName) PID $($proc.Id): $($_.Exception.Message)"
            }
        }
    }

    foreach ($svc in @("SRanipalService", "ViveportDesktopService")) {
        try {
            Stop-Service -Name $svc -Force -ErrorAction Stop
            Write-Host "  stopped service $svc"
        } catch {
            Write-Host "  could not stop service ${svc}: $($_.Exception.Message)"
        }
    }

    try {
        Stop-Service -Name "Tobii VRU02 Runtime" -Force -ErrorAction Stop
        Write-Host "  stopped service Tobii VRU02 Runtime"
    } catch {
        Write-Host "  warning: could not stop Tobii VRU02 Runtime: $($_.Exception.Message)"
    }
}

function Refresh-Coefficient([string]$TargetPath) {
    if ($CoefficientBin) {
        return $CoefficientBin
    }

    Write-Host "Refreshing coefficient through Runtime public API..."
    $runtime = Get-Process -Name Runtime -ErrorAction SilentlyContinue
    if (-not $runtime) {
        Start-Process -FilePath $RuntimeExe -WorkingDirectory $RuntimeDir -WindowStyle Hidden | Out-Null
        Start-Sleep -Seconds 6
    }

    try {
        $coefficientOutput = & $Python -u $CoefficientScript --output $TargetPath 2>&1
        foreach ($line in $coefficientOutput) {
            Write-Host $line
        }
        if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath $TargetPath)) {
            throw "coefficient refresh failed"
        }
        return $TargetPath
    } catch {
        $fallback = Join-Path $RuntimeDir "user_data.dat"
        Write-Host "Coefficient refresh failed; falling back to $fallback"
        return $fallback
    }
}

function Get-RateHz($Rows, [string]$TimestampField, [switch]$Unique) {
    if ($Rows.Count -lt 2) {
        return 0.0
    }
    $values = @($Rows | ForEach-Object { [int64]$_.$TimestampField })
    if ($Unique) {
        $values = @($values | Sort-Object -Unique)
    } else {
        $values = @($values | Sort-Object)
    }
    if ($values.Count -lt 2) {
        return 0.0
    }
    $span = [double]($values[-1] - $values[0])
    if ($span -le 0) {
        return 0.0
    }
    return (($values.Count - 1) * 1000000.0 / $span)
}

function Count-ValidXY($Rows, [string]$XField, [string]$YField) {
    return @($Rows | Where-Object {
        ([double]$_.$XField -ne 0.0) -or ([double]$_.$YField -ne 0.0)
    }).Count
}

function Summarize-Capture([string]$CaptureDir) {
    $imageCsv = Join-Path $CaptureDir "image_frames.csv"
    $gazeCsv = Join-Path $CaptureDir "gaze_samples.csv"
    if (-not ((Test-Path -LiteralPath $imageCsv) -and (Test-Path -LiteralPath $gazeCsv))) {
        throw "Missing capture CSV files under $CaptureDir"
    }

    $images = @(Import-Csv $imageCsv)
    $gazes = @(Import-Csv $gazeCsv)
    $sizes = @($images | ForEach-Object { "$($_.width)x$($_.height)/$($_.size)" } | Sort-Object -Unique)
    $imageStereoHz = Get-RateHz $images "device_timestamp" -Unique
    $imageRowHz = Get-RateHz $images "device_timestamp"
    $gazeHz = Get-RateHz $gazes "timestamp" -Unique

    $validRecommended = Count-ValidXY $gazes "recom_gaze_x" "recom_gaze_y"
    $validLeftGaze = Count-ValidXY $gazes "left_gaze_x" "left_gaze_y"
    $validRightGaze = Count-ValidXY $gazes "right_gaze_x" "right_gaze_y"
    $validLeftPupil = Count-ValidXY $gazes "left_pupil_x" "left_pupil_y"
    $validRightPupil = Count-ValidXY $gazes "right_pupil_x" "right_pupil_y"

    $imageTs = @{}
    foreach ($row in $images) {
        $imageTs[[string]$row.device_timestamp] = $true
    }
    $matched = 0
    foreach ($row in $gazes) {
        if ($imageTs.ContainsKey([string]$row.timestamp)) {
            $matched += 1
        }
    }

    Write-Host ""
    Write-Host "Hi-res current AlgInterface summary"
    Write-Host "  output dir:                 $CaptureDir"
    Write-Host "  image rows:                 $($images.Count)"
    Write-Host "  image sizes:                $($sizes -join ', ')"
    Write-Host ("  image stereo timestamp Hz:  {0:N3}" -f $imageStereoHz)
    Write-Host ("  image callback row Hz:      {0:N3}" -f $imageRowHz)
    Write-Host "  gaze rows:                  $($gazes.Count)"
    Write-Host ("  gaze timestamp Hz:          {0:N3}" -f $gazeHz)
    Write-Host "  gaze/image timestamp hits:  $matched"
    Write-Host "  valid recommended gaze:     $validRecommended"
    Write-Host "  valid left gaze:            $validLeftGaze"
    Write-Host "  valid right gaze:           $validRightGaze"
    Write-Host "  valid left pupil:           $validLeftPupil"
    Write-Host "  valid right pupil:          $validRightPupil"

    $summaryPath = Join-Path $CaptureDir "hires_current_alg_summary.txt"
    @(
        "output_dir=$CaptureDir",
        "image_rows=$($images.Count)",
        "image_sizes=$($sizes -join ',')",
        ("image_stereo_timestamp_hz={0:N6}" -f $imageStereoHz),
        ("image_callback_row_hz={0:N6}" -f $imageRowHz),
        "gaze_rows=$($gazes.Count)",
        ("gaze_timestamp_hz={0:N6}" -f $gazeHz),
        "gaze_image_timestamp_hits=$matched",
        "valid_recommended_gaze=$validRecommended",
        "valid_left_gaze=$validLeftGaze",
        "valid_right_gaze=$validRightGaze",
        "valid_left_pupil=$validLeftPupil",
        "valid_right_pupil=$validRightPupil"
    ) | Set-Content -LiteralPath $summaryPath -Encoding UTF8
    Write-Host "  summary file:               $summaryPath"

    if ($images.Count -eq 0 -or -not ($sizes -contains "640x480/307200")) {
        throw "Validation failed: high-resolution 640x480/307200 eye images were not captured."
    }
    if ($gazes.Count -eq 0) {
        throw "Validation failed: no gaze rows were captured."
    }
    if ($RequireValidSignals -and ($validRecommended -eq 0 -or $validLeftPupil -eq 0 -or $validRightPupil -eq 0)) {
        throw "Validation failed: valid gaze/pupil signals were not observed. Make sure the headset is worn and calibrated."
    }
}

foreach ($path in @($RuntimeDir, $OldBackupDir, $CurrentAlgDir, $CaptureWrapper, $CoefficientScript, $VisualizeScript)) {
    Assert-FileExists $path
}

if (-not $OutputDir) {
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $OutputDir = Join-Path $ScriptDir "hires_current_alg_sync_test_$stamp"
}
$OutputDir = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($OutputDir)
New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null

$runtimeBackup = Join-Path $RuntimeDir ("backup_before_hires_current_alg_" + (Get-Date -Format "yyyyMMdd_HHmmss"))
$coefficientTarget = Join-Path $OutputDir "coefficient.bin"
$captureExitCode = 1

Write-Host "Output directory: $OutputDir"
Write-Host "Runtime backup:   $runtimeBackup"
Write-Host "Combo: old USERSDK.dll + current AlgInterface/support DLLs"

Backup-RuntimeFiles $runtimeBackup
$coefficientPath = Refresh-Coefficient $coefficientTarget

try {
    if (-not $SkipStopCompetingProcesses) {
        Stop-CompetingProcesses
    } else {
        Get-Process -Name Runtime -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
    }
    Start-Sleep -Seconds 2

    Write-Host "Installing hi-res current AlgInterface combo..."
    Install-HiResCurrentAlgCombo

    Write-Host "Starting capture..."
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File $CaptureWrapper `
        -Timeout $Timeout `
        -MaxImages $MaxImages `
        -MaxGaze $MaxGaze `
        -OutputDir $OutputDir `
        -CoefficientBin $coefficientPath `
        -NoRestartRuntime
    $captureExitCode = $LASTEXITCODE
    if ($captureExitCode -ne 0) {
        throw "Capture failed with exit code $captureExitCode"
    }

    Summarize-Capture $OutputDir

    Write-Host ""
    Write-Host "Drawing gaze trajectory..."
    & $Python -u $VisualizeScript $OutputDir
    if ($LASTEXITCODE -ne 0) {
        throw "Gaze visualization failed with exit code $LASTEXITCODE"
    }

    Write-Host ""
    Write-Host "Done."
    Write-Host "Frames:        $OutputDir\frames"
    Write-Host "Image CSV:     $OutputDir\image_frames.csv"
    Write-Host "Gaze CSV:      $OutputDir\gaze_samples.csv"
    Write-Host "Gaze plot:     $OutputDir\gaze_visualization.png"
    exit 0
} finally {
    if (-not $KeepRuntimeCombo) {
        Write-Host "Restoring runtime files from $runtimeBackup"
        Restore-RuntimeFiles $runtimeBackup
    } else {
        Write-Host "Keeping hi-res current AlgInterface combo installed."
    }

    if (-not $NoRestartRuntime) {
        Write-Host "Restarting Runtime.exe"
        Start-Process -FilePath $RuntimeExe -WorkingDirectory $RuntimeDir -WindowStyle Hidden | Out-Null
    }
}
