$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RuntimeDir = "C:\7invensun\aSeeVR_UserSDK\runtime"
$RuntimeExe = Join-Path $RuntimeDir "Runtime.exe"
$SdkConfigDir = Join-Path $RuntimeDir "devices\Droolon\config"
$EyeAlgPath = Join-Path $SdkConfigDir "EyeAlg.ini"
$ProbeDir = Join-Path $ScriptDir "aseevr_fdimage_probe"
$BackupPath = Join-Path $ProbeDir "EyeAlg.ini.before_probe"
$HadExistingMarker = Join-Path $ProbeDir "had_existing_eyealg.txt"
$FdImageDir = Join-Path $SdkConfigDir "armeabi\image\FDImage"
$Python = "C:\Python311\python.exe"
$TestScript = Join-Path $ScriptDir "test_aseevr_eye_image_callback.py"

function Wait-Port {
    param([int]$Port, [double]$TimeoutSeconds = 20.0)
    $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
    while ((Get-Date) -lt $deadline) {
        try {
            $client = New-Object System.Net.Sockets.TcpClient
            $iar = $client.BeginConnect("127.0.0.1", $Port, $null, $null)
            if ($iar.AsyncWaitHandle.WaitOne(500)) {
                $client.EndConnect($iar)
                $client.Close()
                return $true
            }
            $client.Close()
        } catch {
        }
        Start-Sleep -Milliseconds 300
    }
    return $false
}

function Get-RuntimeProcesses {
    Get-Process -Name "Runtime" -ErrorAction SilentlyContinue |
        Where-Object { $_.Path -eq $RuntimeExe }
}

function Restart-Runtime {
    $running = @(Get-RuntimeProcesses)
    foreach ($proc in $running) {
        Write-Host "Stopping Runtime PID $($proc.Id)"
        Stop-Process -Id $proc.Id -Force
    }
    Start-Sleep -Seconds 1

    Write-Host "Starting Runtime from $RuntimeExe"
    Start-Process -FilePath $RuntimeExe -WorkingDirectory $RuntimeDir | Out-Null
    if (-not (Wait-Port -Port 5777 -TimeoutSeconds 20.0)) {
        throw "Runtime did not open port 5777"
    }
}

New-Item -ItemType Directory -Path $ProbeDir -Force | Out-Null
New-Item -ItemType Directory -Path $FdImageDir -Force | Out-Null

$hadExisting = Test-Path -LiteralPath $EyeAlgPath
if ($hadExisting) {
    Copy-Item -LiteralPath $EyeAlgPath -Destination $BackupPath -Force
    Set-Content -LiteralPath $HadExistingMarker -Value "1" -Encoding ascii
    Write-Host "Backed up existing EyeAlg.ini to: $BackupPath"
} else {
    if (Test-Path -LiteralPath $BackupPath) {
        Remove-Item -LiteralPath $BackupPath -Force
    }
    Set-Content -LiteralPath $HadExistingMarker -Value "0" -Encoding ascii
    Write-Host "No existing EyeAlg.ini; probe will remove the temporary file afterward."
}

$before = @{}
Get-ChildItem -LiteralPath $FdImageDir -Recurse -File -ErrorAction SilentlyContinue |
    ForEach-Object { $before[$_.FullName] = $_.LastWriteTimeUtc }

$eyeAlg = @'
[ALG_QUICK_SETTING]
ID=1
Enabled=1
AlgAPI=0
APIIndex=1
NeedInput=1
InputID=6
RunType=0
RunInterval=0
SaveResult=1
LimitRunInterval=0
LimitRunCount=0
IsNeedResContinue=0

[ALG_BLINK_SETTING]
ID=2
Enabled=1
AlgAPI=0
APIIndex=2
NeedInput=0
InputID=0
RunType=0
RunInterval=0
SaveResult=1
LimitRunInterval=0
LimitRunCount=0
IsNeedResContinue=0

[ALG_BRIGHT_SETTING]
ID=3
Enabled=1
AlgAPI=2
APIIndex=3
NeedInput=0
InputID=0
RunType=1
RunInterval=3000
SaveResult=1
LimitRunInterval=0
LimitRunCount=0
IsNeedResContinue=0

[ALG_EQUIP_SETTING]
ID=4
Enabled=0
AlgAPI=0
APIIndex=0
NeedInput=0
InputID=0
RunType=0
RunInterval=0
SaveResult=0
LimitRunInterval=0
LimitRunCount=0
IsNeedResContinue=0

[ALG_GLASS_SETTING]
ID=5
Enabled=0
AlgAPI=0
APIIndex=0
NeedInput=0
InputID=0
RunType=0
RunInterval=0
SaveResult=0
LimitRunInterval=0
LimitRunCount=0
IsNeedResContinue=0

[ALG_SLOW_SETTING]
ID=6
Enabled=1
AlgAPI=1
APIIndex=0
NeedInput=0
InputID=0
RunType=2
RunInterval=3000
SaveResult=1
LimitRunInterval=0
LimitRunCount=0
IsNeedResContinue=0

[ALG_EYELID_SETTING]
ID=7
Enabled=1
AlgAPI=0
APIIndex=6
NeedInput=0
InputID=0
RunType=0
RunInterval=0
SaveResult=1
LimitRunInterval=0
LimitRunCount=0
IsNeedResContinue=0

[ALG_OUTLINE_SETTING]
ID=8
Enabled=0
AlgAPI=3
APIIndex=0
NeedInput=0
InputID=0
RunType=2
RunInterval=3000
SaveResult=0
LimitRunInterval=500
LimitRunCount=10
IsNeedResContinue=0

[ALG_API_QUICK_TRACKING_SETTING]
ID=0
Enabled=1
RunThread=0
RunSingleton=0

[ALG_API_SLOW_ONE_SETTING]
ID=1
Enabled=1
RunThread=1
RunSingleton=0

[ALG_API_SLOW_TWO_SETTING]
ID=2
Enabled=1
RunThread=1
RunSingleton=0

[ALG_API_GPU_SETTING]
ID=3
Enabled=0
RunThread=1
RunSingleton=0
'@

try {
    Set-Content -LiteralPath $EyeAlgPath -Value $eyeAlg -Encoding ascii
    Write-Host "Wrote temporary EyeAlg.ini:"
    Get-Content -LiteralPath $EyeAlgPath

    Restart-Runtime

    Write-Host "Running callback probe"
    & $Python -u $TestScript --timeout 15 --max-frames 4
    $testExit = $LASTEXITCODE
    Write-Host "Callback probe exit code: $testExit"

    Start-Sleep -Seconds 2
    Write-Host "New or modified FDImage files:"
    $changed = Get-ChildItem -LiteralPath $FdImageDir -Recurse -File -ErrorAction SilentlyContinue |
        Where-Object { -not $before.ContainsKey($_.FullName) -or $before[$_.FullName] -ne $_.LastWriteTimeUtc } |
        Sort-Object LastWriteTime |
        Select-Object FullName,Length,LastWriteTime
    $changed | Format-Table -AutoSize
} finally {
    Write-Host "Restoring EyeAlg.ini state"
    if ($hadExisting) {
        Copy-Item -LiteralPath $BackupPath -Destination $EyeAlgPath -Force
    } elseif (Test-Path -LiteralPath $EyeAlgPath) {
        Remove-Item -LiteralPath $EyeAlgPath -Force
    }
    Restart-Runtime
}

Write-Host "Probe artifacts: $ProbeDir"
