$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$WorkspaceRoot = Resolve-Path (Join-Path $ScriptDir "..")
$RuntimeDir = "C:\7invensun\aSeeVR_UserSDK\runtime"
$RuntimeExe = Join-Path $RuntimeDir "Runtime.exe"
$ConfigPath = Join-Path $RuntimeDir "config.xml"
$ProbeDir = Join-Path $ScriptDir "aseevr_runtime_video_probe"
$VideoDir = Join-Path $ProbeDir "eye_video_out"
$BackupPath = Join-Path $ProbeDir "config.xml.before_probe"
$Python = "C:\Python311\python.exe"
$TestScript = Join-Path $ScriptDir "test_aseevr_eye_image_callback.py"

function Wait-Port {
    param([int]$Port, [double]$TimeoutSeconds = 15.0)
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

New-Item -ItemType Directory -Path $ProbeDir -Force | Out-Null
New-Item -ItemType Directory -Path $VideoDir -Force | Out-Null
Copy-Item -LiteralPath $ConfigPath -Destination $BackupPath -Force

$wasRunning = @(Get-RuntimeProcesses).Count -gt 0
Write-Host "Runtime was running: $wasRunning"
Write-Host "Backed up config to: $BackupPath"

try {
    [xml]$xml = Get-Content -LiteralPath $ConfigPath
    $root = $xml.aSeeVR_runtime_config

    $portNode = $root.SelectSingleNode("port")
    $insertAfter = $portNode
    foreach ($name in @("eye_video_path", "eye_video_fps")) {
        $existing = $root.SelectSingleNode($name)
        if ($null -eq $existing) {
            $node = $xml.CreateElement($name)
            [void]$root.InsertAfter($node, $insertAfter)
            $insertAfter = $node
        }
    }

    $root.SelectSingleNode("eye_video_path").InnerText = [string]$VideoDir
    $root.SelectSingleNode("eye_video_fps").InnerText = "30"
    $xml.Save($ConfigPath)
    Write-Host "Wrote temporary video config:"
    Get-Content -LiteralPath $ConfigPath

    $running = @(Get-RuntimeProcesses)
    foreach ($proc in $running) {
        Write-Host "Stopping Runtime PID $($proc.Id)"
        Stop-Process -Id $proc.Id -Force
    }
    Start-Sleep -Seconds 1

    Write-Host "Starting Runtime from $RuntimeExe"
    $proc = Start-Process -FilePath $RuntimeExe -WorkingDirectory $RuntimeDir -WindowStyle Hidden -PassThru
    if (-not (Wait-Port -Port 5777 -TimeoutSeconds 20.0)) {
        throw "Runtime did not open port 5777"
    }

    Write-Host "Running callback probe"
    & $Python -u $TestScript --timeout 8 --max-frames 4
    $testExit = $LASTEXITCODE
    Write-Host "Callback probe exit code: $testExit"

    Start-Sleep -Seconds 2
    Write-Host "Video output files:"
    Get-ChildItem -LiteralPath $VideoDir -Recurse -File -ErrorAction SilentlyContinue |
        Select-Object FullName,Length,LastWriteTime |
        Format-Table -AutoSize
} finally {
    Write-Host "Restoring original config"
    Copy-Item -LiteralPath $BackupPath -Destination $ConfigPath -Force

    $running = @(Get-RuntimeProcesses)
    foreach ($proc in $running) {
        Write-Host "Stopping probe Runtime PID $($proc.Id)"
        Stop-Process -Id $proc.Id -Force
    }

    if ($wasRunning) {
        Write-Host "Restarting Runtime with restored config"
        Start-Process -FilePath $RuntimeExe -WorkingDirectory $RuntimeDir -WindowStyle Hidden | Out-Null
        [void](Wait-Port -Port 5777 -TimeoutSeconds 20.0)
    }
}

Write-Host "Probe artifacts: $ProbeDir"
