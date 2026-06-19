$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RuntimeDir = "C:\7invensun\aSeeVR_UserSDK\runtime"
$RuntimeExe = Join-Path $RuntimeDir "Runtime.exe"
$RuntimeConfig = Join-Path $RuntimeDir "config.xml"
$SdkConfigDir = Join-Path $RuntimeDir "devices\Droolon\config"
$ProbeDir = Join-Path $ScriptDir "aseevr_gaze_image_probe"
$SavePath = Join-Path $ProbeDir "save_path"
$Python = "C:\Python311\python.exe"
$TestScript = Join-Path $ScriptDir "test_aseevr_eye_image_callback.py"

$ConfigFiles = @("EyeAlg.ini", "EyeTracking.ini", "GazeCfg.ini")

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

function Snapshot-Files {
    param([string[]]$Roots)
    $snapshot = @{}
    foreach ($root in $Roots) {
        if (Test-Path -LiteralPath $root) {
            Get-ChildItem -LiteralPath $root -Recurse -File -ErrorAction SilentlyContinue |
                ForEach-Object { $snapshot[$_.FullName] = "$($_.Length)|$($_.LastWriteTimeUtc.Ticks)" }
        }
    }
    return $snapshot
}

function Show-ChangedFiles {
    param([hashtable]$Before, [string[]]$Roots)
    $changed = @()
    foreach ($root in $Roots) {
        if (Test-Path -LiteralPath $root) {
            $changed += Get-ChildItem -LiteralPath $root -Recurse -File -ErrorAction SilentlyContinue |
                Where-Object {
                    $value = "$($_.Length)|$($_.LastWriteTimeUtc.Ticks)"
                    -not $Before.ContainsKey($_.FullName) -or $Before[$_.FullName] -ne $value
                } |
                Sort-Object LastWriteTime |
                Select-Object FullName,Length,LastWriteTime
        }
    }
    $changed | Format-Table -AutoSize
}

New-Item -ItemType Directory -Path $ProbeDir -Force | Out-Null
New-Item -ItemType Directory -Path $SavePath -Force | Out-Null

$RuntimeConfigBackup = Join-Path $ProbeDir "config.xml.before_probe"
Copy-Item -LiteralPath $RuntimeConfig -Destination $RuntimeConfigBackup -Force

$existing = @{}
foreach ($name in $ConfigFiles) {
    $path = Join-Path $SdkConfigDir $name
    $backup = Join-Path $ProbeDir "$name.before_probe"
    if (Test-Path -LiteralPath $path) {
        Copy-Item -LiteralPath $path -Destination $backup -Force
        $existing[$name] = $true
        Write-Host "Backed up existing $name"
    } else {
        if (Test-Path -LiteralPath $backup) {
            Remove-Item -LiteralPath $backup -Force
        }
        $existing[$name] = $false
        Write-Host "No existing $name; probe will remove temporary file afterward."
    }
}

$rootsToWatch = @(
    $SavePath,
    $SdkConfigDir,
    (Join-Path $SdkConfigDir "armeabi\image")
)
$before = Snapshot-Files -Roots $rootsToWatch

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

$eyeTracking = @'
[GAZESETTING]
Enabled=1
eyeType=1
fdimg=2
fdimgline=1

[FD_QUICK_SETTING]
Enabled=1
EquipCheck=0
LightCheck=1
GlassCheck=0
BlinkCheck=1
NeedSlowFirst=2
AutoSlowFirstInterval=3000
NeedSlowSecond=1
AutoSlowSecondInterval=3000
NeedSlowThird=0
AutoSlowThirdInterval=0

[FD_SLOW_FIRST_SETTING]
Enabled=1
AlgSlowType=1
AlgGPUType=0
AlgMode=0
SaveResult=1

[FD_SLOW_SECOND_SETTING]
Enabled=1
AlgSlowType=2
AlgGPUType=0
AlgMode=0
SaveResult=1

[FD_SLOW_THIRD_SETTING]
Enabled=0
AlgSlowType=0
AlgGPUType=1
AlgMode=1
SaveResult=1
'@

$gazeCfg = @'
[METHOD]
DeviceType=1
Type=5
MethodNum=1
TwoType=5

[SMOOTH]
Enable=1
Th=0.015
Ex=-0.01
TimeLimit=1300
Fast=0
Motion=1.4

[SETTING]
check=1
checkRadius=1
minX=-5
maxX=5
minY=-5
maxY=5
saveImageMode=1
'@

try {
    [xml]$xml = Get-Content -LiteralPath $RuntimeConfig
    $root = $xml.aSeeVR_runtime_config
    $savePathNode = $root.SelectSingleNode("save_path")
    if ($null -eq $savePathNode) {
        $savePathNode = $xml.CreateElement("save_path")
        [void]$root.InsertAfter($savePathNode, $root.SelectSingleNode("port"))
    }
    $savePathNode.InnerText = [string]$SavePath
    $xml.Save($RuntimeConfig)

    Set-Content -LiteralPath (Join-Path $SdkConfigDir "EyeAlg.ini") -Value $eyeAlg -Encoding ascii
    Set-Content -LiteralPath (Join-Path $SdkConfigDir "EyeTracking.ini") -Value $eyeTracking -Encoding ascii
    Set-Content -LiteralPath (Join-Path $SdkConfigDir "GazeCfg.ini") -Value $gazeCfg -Encoding ascii

    Write-Host "Temporary save_path: $SavePath"
    Write-Host "Temporary config files written under: $SdkConfigDir"

    Restart-Runtime

    Write-Host "Running callback probe"
    & $Python -u $TestScript --timeout 20 --max-frames 4
    $testExit = $LASTEXITCODE
    Write-Host "Callback probe exit code: $testExit"

    Start-Sleep -Seconds 2
    Write-Host "New or modified files under watched roots:"
    Show-ChangedFiles -Before $before -Roots $rootsToWatch
} finally {
    Write-Host "Restoring Runtime config and SDK config files"
    Copy-Item -LiteralPath $RuntimeConfigBackup -Destination $RuntimeConfig -Force
    foreach ($name in $ConfigFiles) {
        $path = Join-Path $SdkConfigDir $name
        $backup = Join-Path $ProbeDir "$name.before_probe"
        if ($existing[$name]) {
            Copy-Item -LiteralPath $backup -Destination $path -Force
        } elseif (Test-Path -LiteralPath $path) {
            Remove-Item -LiteralPath $path -Force
        }
    }
    Restart-Runtime
}

Write-Host "Probe artifacts: $ProbeDir"
