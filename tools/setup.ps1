param([string]$Source)
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$local = Join-Path $repo '.local'
New-Item -ItemType Directory -Path $local -Force | Out-Null
$config = Join-Path $local 'config.json'
if (Test-Path -LiteralPath $config) { Write-Host 'Local configuration already exists.'; exit 0 }
if (-not $Source) { $Source = Read-Host 'Original project folder (read-only source)' }
if (-not (Test-Path -LiteralPath (Join-Path $Source 'Gimbal_STLink.uvprojx'))) {
    throw 'Select the folder containing Gimbal_STLink.uvprojx.'
}
$settings = @{
    source = (Resolve-Path -LiteralPath $Source).Path
    backup_root = 'D:\嵌入式Git备份\gimbal-inspection-rover'
    remote = 'https://github.com/niceworkkiimm-ai/gimbal-inspection-rover.git'
}
$settings | ConvertTo-Json | Set-Content -LiteralPath $config -Encoding UTF8
Write-Host 'Saved private configuration in .local/config.json.'
