param(
    [double]$Timeout = 10.0,
    [int]$MaxImages = 120,
    [int]$MaxGaze = 2000,
    [string]$OutputDir = "",
    [string]$CoefficientBin = "",
    [switch]$NoTracking,
    [switch]$NoRestartRuntime
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RuntimeExe = "C:\7invensun\aSeeVR_UserSDK\runtime\Runtime.exe"
$RuntimeDir = Split-Path $RuntimeExe
$Python = "C:\Python311\python.exe"
$CaptureScript = Join-Path $ScriptDir "probe_usersdk_sync_callbacks.py"
$CoefficientScript = Join-Path $ScriptDir "get_aseevr_coefficient.py"

if (-not $OutputDir) {
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $OutputDir = Join-Path $ScriptDir "usersdk_sync_test_$stamp"
}

$OutputDir = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($OutputDir)
New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null

if (-not $CoefficientBin) {
    $CoefficientBin = Join-Path $OutputDir "coefficient.bin"
    try {
        Write-Host "Refreshing coefficient through Runtime public API..."
        & $Python -u $CoefficientScript --output $CoefficientBin
        if ($LASTEXITCODE -ne 0 -or -not (Test-Path $CoefficientBin)) {
            throw "coefficient refresh failed"
        }
    } catch {
        $fallback = "C:\7invensun\aSeeVR_UserSDK\runtime\user_data.dat"
        Write-Host "Coefficient refresh failed; falling back to $fallback"
        $CoefficientBin = $fallback
    }
}

Write-Host "Output directory: $OutputDir"
if ($NoTracking) {
    Write-Host "Tracking: disabled"
} else {
    Write-Host "Tracking coefficient: $CoefficientBin"
}
Write-Host "Stopping Runtime.exe so USERSDK can open the camera directly..."

try {
    Get-Process -Name Runtime -ErrorAction SilentlyContinue |
        ForEach-Object {
            Write-Host "Stopping Runtime PID $($_.Id)"
            Stop-Process -Id $_.Id -Force
        }
    Start-Sleep -Seconds 1

    $captureArgs = @(
        "-u", $CaptureScript,
        "--timeout", $Timeout,
        "--max-images", $MaxImages,
        "--max-gaze", $MaxGaze,
        "--output-dir", $OutputDir
    )
    if ($NoTracking) {
        $captureArgs += @("--tracking-mode", "none")
    } else {
        $captureArgs += @(
            "--coefficient-bin", $CoefficientBin,
            "--coefficient-layout", "split1024-sized",
            "--tracking-mode", "start-tracking",
            "--tracking-eyes", "both"
        )
    }

    & $Python @captureArgs
    $exitCode = $LASTEXITCODE

    $imageCsv = Join-Path $OutputDir "image_frames.csv"
    $gazeCsv = Join-Path $OutputDir "gaze_samples.csv"
    if ((Test-Path $imageCsv) -and (Test-Path $gazeCsv)) {
        $images = @(Import-Csv $imageCsv)
        $gazes = @(Import-Csv $gazeCsv)
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
        Write-Host "Summary"
        Write-Host "  image rows: $($images.Count)"
        Write-Host "  gaze rows:  $($gazes.Count)"
        Write-Host "  gaze timestamps also present in saved image CSV: $matched"
        if ($images.Count -gt 0) {
            Write-Host "  first image timestamp: $($images[0].device_timestamp)"
        }
        if ($gazes.Count -gt 0) {
            Write-Host "  first gaze timestamp:  $($gazes[0].timestamp)"
            Write-Host "  first gaze point:      ($($gazes[0].recom_gaze_x), $($gazes[0].recom_gaze_y))"
            Write-Host "  first left pupil:      ($($gazes[0].left_pupil_x), $($gazes[0].left_pupil_y))"
            Write-Host "  first right pupil:     ($($gazes[0].right_pupil_x), $($gazes[0].right_pupil_y))"

            $validRecomGaze = @($gazes | Where-Object {
                ([double]$_.recom_gaze_x -ne 0.0) -or ([double]$_.recom_gaze_y -ne 0.0)
            })
            $validLeftGaze = @($gazes | Where-Object {
                ([double]$_.left_gaze_x -ne 0.0) -or ([double]$_.left_gaze_y -ne 0.0)
            })
            $validRightGaze = @($gazes | Where-Object {
                ([double]$_.right_gaze_x -ne 0.0) -or ([double]$_.right_gaze_y -ne 0.0)
            })
            $validLeftPupil = @($gazes | Where-Object {
                ([double]$_.left_pupil_x -ne 0.0) -or ([double]$_.left_pupil_y -ne 0.0)
            })
            $validRightPupil = @($gazes | Where-Object {
                ([double]$_.right_pupil_x -ne 0.0) -or ([double]$_.right_pupil_y -ne 0.0)
            })

            Write-Host "  valid recommended gaze rows: $($validRecomGaze.Count)"
            Write-Host "  valid left gaze rows:        $($validLeftGaze.Count)"
            Write-Host "  valid right gaze rows:       $($validRightGaze.Count)"
            Write-Host "  valid left pupil rows:       $($validLeftPupil.Count)"
            Write-Host "  valid right pupil rows:      $($validRightPupil.Count)"

            if ($validRecomGaze.Count -gt 0) {
                $row = $validRecomGaze[0]
                Write-Host "  first valid recommended gaze: ts=$($row.timestamp) point=($($row.recom_gaze_x), $($row.recom_gaze_y)) recommend=$($row.recommend)"
            }
            if ($validLeftPupil.Count -gt 0) {
                $row = $validLeftPupil[0]
                Write-Host "  first valid left pupil:       ts=$($row.timestamp) center=($($row.left_pupil_x), $($row.left_pupil_y)) diameter_mm=$($row.left_pupil_diameter_mm)"
            }
            if ($validRightPupil.Count -gt 0) {
                $row = $validRightPupil[0]
                Write-Host "  first valid right pupil:      ts=$($row.timestamp) center=($($row.right_pupil_x), $($row.right_pupil_y)) diameter_mm=$($row.right_pupil_diameter_mm)"
            }
        }
    }

    Write-Host ""
    Write-Host "Capture exit code: $exitCode"
    Write-Host "Frames: $OutputDir\frames"
    Write-Host "Image CSV: $imageCsv"
    Write-Host "Gaze CSV:  $gazeCsv"
    exit $exitCode
} finally {
    if (-not $NoRestartRuntime) {
        Write-Host "Restarting Runtime.exe"
        Start-Process -FilePath $RuntimeExe -WorkingDirectory $RuntimeDir -WindowStyle Hidden | Out-Null
    }
}
