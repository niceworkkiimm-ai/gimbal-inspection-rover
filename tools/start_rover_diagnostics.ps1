param([switch]$NoBrowser)

$ErrorActionPreference = 'Stop'
$statusUrl = 'http://127.0.0.1:8767/api/status'
function Test-Recorder {
    try {
        $result = Invoke-RestMethod -Uri $statusUrl -TimeoutSec 1
        return ($null -ne $result.latest.recorder -and $null -ne $result.PSObject.Properties['cameraHost'])
    } catch {
        return $false
    }
}

if (-not (Test-Recorder)) {
    $python = (Get-Command python.exe -ErrorAction SilentlyContinue).Source
    $pythonArgs = @()
    if (-not $python) {
        $python = (Get-Command py.exe -ErrorAction Stop).Source
        $pythonArgs = @('-3')
    }
    $script = Join-Path $PSScriptRoot 'rover_diagnostics.py'
    $logDir = 'D:\RoverDiagnostics\logs'
    New-Item -ItemType Directory -Path $logDir -Force | Out-Null
    $process = Start-Process -FilePath $python -ArgumentList ($pythonArgs + @('-B', ('"' + $script + '"'))) `
        -RedirectStandardOutput (Join-Path $logDir 'recorder-stdout.log') `
        -RedirectStandardError (Join-Path $logDir 'recorder-stderr.log') -WindowStyle Hidden -PassThru
    $ready = $false
    for ($attempt = 0; $attempt -lt 15; $attempt++) {
        if (Test-Recorder) {
            $ready = $true
            break
        }
        if ($process.HasExited) { break }
        Start-Sleep -Milliseconds 300
    }
    if (-not $ready) {
        Get-Content (Join-Path $logDir 'recorder-stderr.log') -Tail 12 -ErrorAction SilentlyContinue
        throw "Rover diagnostics did not start. Check $logDir\recorder-stderr.log"
    }
}

if (-not $NoBrowser) { Start-Process 'http://127.0.0.1:8767/' }
