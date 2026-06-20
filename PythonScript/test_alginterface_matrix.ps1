param(
    [int]$Timeout = 6,
    [int]$MaxImages = 80,
    [int]$MaxGaze = 1500,
    [string]$RuntimeDir = "C:\7invensun\aSeeVR_UserSDK\runtime",
    [string]$OldDir = "C:\7invensun\aSeeVR_UserSDK\runtime\backup_before_alginterface_20260620_111308",
    [string]$NewDir = "C:\Third_Party\AlgInterface",
    [string]$OutputRoot = "C:\Third_Party\PythonEyetracker\PythonScript"
)

$ErrorActionPreference = "Stop"

$Files = @(
    "AlgInterface.dll",
    "CertPlatform.dll",
    "libeay32.dll",
    "SdkEvent.dll",
    "smoothAbout.dll",
    "sqlite3.dll",
    "USERSDK.dll",
    "zlib1.dll"
)

$Combos = @(
    @{ Name = "new_all"; Usersdk = "new"; Alg = "new"; Support = "new" },
    @{ Name = "old_all"; Usersdk = "old"; Alg = "old"; Support = "old" },
    @{ Name = "new_usersdk_old_alg"; Usersdk = "new"; Alg = "old"; Support = "new" },
    @{ Name = "old_usersdk_new_alg"; Usersdk = "old"; Alg = "new"; Support = "old" },
    @{ Name = "new_usersdk_old_alg_old_support"; Usersdk = "new"; Alg = "old"; Support = "old" },
    @{ Name = "old_usersdk_new_alg_new_support"; Usersdk = "old"; Alg = "new"; Support = "new" }
)

function Stop-Runtime {
    $runtimeProcs = Get-Process -Name Runtime -ErrorAction SilentlyContinue
    foreach ($p in $runtimeProcs) {
        Write-Host "Stopping Runtime PID $($p.Id)"
        Stop-Process -Id $p.Id -Force
    }
    Start-Sleep -Milliseconds 800
}

function Source-Path([string]$which, [string]$name) {
    if ($which -eq "old") {
        return Join-Path $OldDir $name
    }
    return Join-Path $NewDir $name
}

function Install-Combo($combo) {
    Stop-Runtime
    foreach ($name in $Files) {
        $which = $combo.Support
        if ($name -eq "USERSDK.dll") {
            $which = $combo.Usersdk
        } elseif ($name -eq "AlgInterface.dll") {
            $which = $combo.Alg
        }
        $src = Source-Path $which $name
        $dst = Join-Path $RuntimeDir $name
        Copy-Item -LiteralPath $src -Destination $dst -Force
    }
}

function Summarize-Capture([string]$dir) {
    $script = @'
import csv
import json
import statistics
import sys
from pathlib import Path

base = Path(sys.argv[1])
img_path = base / "image_frames.csv"
gaze_path = base / "gaze_samples.csv"
imgs = list(csv.DictReader(img_path.open(newline="", encoding="utf-8"))) if img_path.exists() else []
gazes = list(csv.DictReader(gaze_path.open(newline="", encoding="utf-8"))) if gaze_path.exists() else []

def ff(value):
    try:
        return float(value)
    except Exception:
        return 0.0

def valid(rows, x, y):
    return [r for r in rows if abs(ff(r.get(x, "0"))) > 1e-9 or abs(ff(r.get(y, "0"))) > 1e-9]

def rate(rows, field):
    if len(rows) < 2:
        return None
    ts = [int(r[field]) for r in rows]
    span = (max(ts) - min(ts)) / 1000000.0
    dts = [(b - a) / 1000.0 for a, b in zip(ts, ts[1:]) if b > a]
    return {
        "rows": len(rows),
        "unique_ts": len(set(ts)),
        "span_s": span,
        "hz": (len(rows) - 1) / span if span > 0 else 0,
        "median_dt_ms": statistics.median(dts) if dts else 0,
        "first_ts": min(ts),
        "last_ts": max(ts),
    }

summary = {
    "output_dir": str(base),
    "image_rows": len(imgs),
    "gaze_rows": len(gazes),
    "image_size_set": sorted({(r.get("width"), r.get("height"), r.get("size")) for r in imgs}),
    "image_eyes": sorted({r.get("eye") for r in imgs}),
    "image_rate_saved_rows": rate(imgs, "device_timestamp") if imgs else None,
    "gaze_rate_all_rows": rate(gazes, "timestamp") if gazes else None,
}

if imgs:
    unique_ts = sorted({int(r["device_timestamp"]) for r in imgs})
    if len(unique_ts) > 1:
        span = (unique_ts[-1] - unique_ts[0]) / 1000000.0
        summary["image_rate_stereo_ts_hz"] = (len(unique_ts) - 1) / span if span > 0 else 0
    for eye in sorted({r.get("eye") for r in imgs}):
        summary[f"image_eye_{eye}_rate"] = rate([r for r in imgs if r.get("eye") == eye], "device_timestamp")

if gazes:
    fields = [
        ("recommended_gaze", "recom_gaze_x", "recom_gaze_y"),
        ("left_gaze", "left_gaze_x", "left_gaze_y"),
        ("right_gaze", "right_gaze_x", "right_gaze_y"),
        ("left_pupil", "left_pupil_x", "left_pupil_y"),
        ("right_pupil", "right_pupil_x", "right_pupil_y"),
    ]
    for label, x, y in fields:
        rows = valid(gazes, x, y)
        entry = {"valid": len(rows), "percent": len(rows) * 100.0 / len(gazes)}
        if rows:
            xs = [ff(r[x]) for r in rows]
            ys = [ff(r[y]) for r in rows]
            entry.update({
                "x_min": min(xs),
                "x_max": max(xs),
                "y_min": min(ys),
                "y_max": max(ys),
                "rate": rate(rows, "timestamp"),
            })
        summary[label] = entry
    counts = {}
    for r in gazes:
        counts[r.get("recommend", "")] = counts.get(r.get("recommend", ""), 0) + 1
    summary["recommend_counts"] = counts

print(json.dumps(summary, ensure_ascii=False, indent=2))
'@
    $tmp = Join-Path $env:TEMP "summarize_usersdk_capture.py"
    Set-Content -LiteralPath $tmp -Value $script -Encoding UTF8
    & C:\Python311\python.exe $tmp $dir
}

$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$matrixDir = Join-Path $OutputRoot "alginterface_matrix_$stamp"
New-Item -ItemType Directory -Path $matrixDir | Out-Null
$summaryRows = @()

foreach ($combo in $Combos) {
    Write-Host "=== combo $($combo.Name) ==="
    Install-Combo $combo
    $log = Join-Path $matrixDir "$($combo.Name).log"
    $before = Get-Date
    $oldErrorActionPreference = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $OutputRoot "capture_usersdk_sync_test.ps1") -Timeout $Timeout -MaxImages $MaxImages -MaxGaze $MaxGaze *> $log
    $captureExitCode = $LASTEXITCODE
    $ErrorActionPreference = $oldErrorActionPreference
    $after = Get-Date
    $captureDir = Get-ChildItem -Path $OutputRoot -Directory -Filter "usersdk_sync_test_*" |
        Where-Object { $_.LastWriteTime -ge $before.AddSeconds(-2) -and $_.LastWriteTime -le $after.AddSeconds(30) } |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1
    if (-not $captureDir) {
        $captureDir = Get-ChildItem -Path $OutputRoot -Directory -Filter "usersdk_sync_test_*" |
            Sort-Object LastWriteTime -Descending |
            Select-Object -First 1
    }
    $jsonText = Summarize-Capture $captureDir.FullName
    $jsonPath = Join-Path $matrixDir "$($combo.Name).summary.json"
    Set-Content -LiteralPath $jsonPath -Value $jsonText -Encoding UTF8
    $obj = $jsonText | ConvertFrom-Json
    $summaryRows += [PSCustomObject]@{
        combo = $combo.Name
        output_dir = $captureDir.FullName
        image_rows = $obj.image_rows
        gaze_rows = $obj.gaze_rows
        image_size_set = (($obj.image_size_set | ForEach-Object { "$($_[0])x$($_[1])/$($_[2])" }) -join ";")
        image_stereo_hz = $obj.image_rate_stereo_ts_hz
        gaze_all_hz = $obj.gaze_rate_all_rows.hz
        recommended_valid = $obj.recommended_gaze.valid
        recommended_percent = $obj.recommended_gaze.percent
        left_pupil_valid = $obj.left_pupil.valid
        left_pupil_percent = $obj.left_pupil.percent
        right_pupil_valid = $obj.right_pupil.valid
        right_pupil_percent = $obj.right_pupil.percent
        exit_code = $captureExitCode
    }
    $summaryRows[-1] | Format-List
}

$summaryCsv = Join-Path $matrixDir "matrix_summary.csv"
$summaryRows | Export-Csv -LiteralPath $summaryCsv -NoTypeInformation -Encoding UTF8
Write-Host "matrix_dir=$matrixDir"
Write-Host "summary_csv=$summaryCsv"
