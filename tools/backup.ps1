$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$env:PYTHONDONTWRITEBYTECODE = '1'
$env:PYTHONIOENCODING = 'utf-8'
Set-Location -LiteralPath $repo
if (-not (Test-Path -LiteralPath '.local/config.json')) { & "$PSScriptRoot/setup.ps1" }
$python = if (Test-Path 'D:\Anaconda3\python.exe') { 'D:\Anaconda3\python.exe' } else { (Get-Command python -ErrorAction Stop).Source }
$message = Read-Host 'Describe the verified changes for this Git commit'
if ([string]::IsNullOrWhiteSpace($message)) { throw 'A meaningful commit message is required.' }
& $python -B "$PSScriptRoot/project_sync.py" publish --message $message
if ($LASTEXITCODE -ne 0) { throw 'Backup/push did not complete. Review the error above; local commits are retained.' }
Write-Host 'Backup and GitHub commit verification completed.' -ForegroundColor Green
