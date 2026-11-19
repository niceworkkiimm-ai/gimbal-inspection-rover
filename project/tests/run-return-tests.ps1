param([string]$BuildDirectory = (Join-Path $PSScriptRoot '..\test-build'))
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
New-Item -ItemType Directory -Path $BuildDirectory -Force | Out-Null
$testExe = Join-Path (Resolve-Path $BuildDirectory).Path 'return-path-test.exe'
& gcc -std=c99 -Wall -Wextra -Werror -I (Join-Path $projectRoot 'Hardware') `
    (Join-Path $PSScriptRoot 'return-path.test.c') `
    (Join-Path $projectRoot 'Hardware\ReturnPath.c') `
    (Join-Path $projectRoot 'Hardware\WheelSpeed.c') -o $testExe
if ($LASTEXITCODE -ne 0) { throw 'Host C test build failed' }
& $testExe
if ($LASTEXITCODE -ne 0) { throw 'Return controller tests failed' }
& node --test (Join-Path $PSScriptRoot 'return-panel.test.cjs')
if ($LASTEXITCODE -ne 0) { throw 'Return browser tests failed' }
& node (Join-Path $PSScriptRoot 'route-snapshot.test.cjs')
if ($LASTEXITCODE -ne 0) { throw 'Snapshot regression failed' }
& node (Join-Path $PSScriptRoot 'rover-demo.test.cjs')
if ($LASTEXITCODE -ne 0) { throw 'Demo regression failed' }
