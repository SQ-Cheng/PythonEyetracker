param(
    [double]$Timeout = 8.0,
    [int]$MaxFrames = 120,
    [string]$OutputDir = ""
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RuntimeExe = "C:\7invensun\aSeeVR_UserSDK\runtime\Runtime.exe"
$RuntimeDir = Split-Path $RuntimeExe
$Python = "C:\Python311\python.exe"
$CaptureScript = Join-Path $ScriptDir "probe_usersdk_camera_callback.py"

if (-not $OutputDir) {
    $OutputDir = Join-Path $ScriptDir "usersdk_camera_callback_frames"
}

try {
    Get-Process -Name Runtime -ErrorAction SilentlyContinue |
        Where-Object { $_.Path -eq $RuntimeExe } |
        ForEach-Object {
            Write-Host "Stopping Runtime PID $($_.Id)"
            Stop-Process -Id $_.Id -Force
        }
    Start-Sleep -Seconds 1

    & $Python -u $CaptureScript --timeout $Timeout --max-frames $MaxFrames --output-dir $OutputDir
    $exitCode = $LASTEXITCODE
    Write-Host "Capture exit code: $exitCode"
    exit $exitCode
} finally {
    Write-Host "Restarting Runtime"
    Start-Process -FilePath $RuntimeExe -WorkingDirectory $RuntimeDir | Out-Null
}
