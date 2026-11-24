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
$scanExe = Join-Path (Resolve-Path $BuildDirectory).Path 'scan-echo-test.exe'
& gcc -std=c99 -Wall -Wextra -Werror -I (Join-Path $projectRoot 'Hardware') (Join-Path $PSScriptRoot 'scan-echo.test.c') -o $scanExe
if ($LASTEXITCODE -ne 0) { throw 'Scan echo test build failed' }
& $scanExe
if ($LASTEXITCODE -ne 0) { throw 'Scan echo tests failed' }
$strafeExe = Join-Path (Resolve-Path $BuildDirectory).Path 'strafe-test.exe'
& gcc -std=c99 -Wall -Wextra -Werror -I (Join-Path $projectRoot 'Hardware') (Join-Path $PSScriptRoot 'strafe.test.c') (Join-Path $projectRoot 'Hardware/CarLogic.c') -o $strafeExe
if ($LASTEXITCODE -ne 0) { throw 'Strafe test build failed' }
& $strafeExe
if ($LASTEXITCODE -ne 0) { throw 'Strafe regression failed' }
& node (Join-Path $PSScriptRoot 'parked-scan.test.cjs')
if ($LASTEXITCODE -ne 0) { throw 'Parked scan regression failed' }
& node --test (Join-Path $PSScriptRoot 'return-panel.test.cjs')
if ($LASTEXITCODE -ne 0) { throw 'Return browser tests failed' }
& node (Join-Path $PSScriptRoot 'route-snapshot.test.cjs')
if ($LASTEXITCODE -ne 0) { throw 'Snapshot regression failed' }
& node (Join-Path $PSScriptRoot 'rover-demo.test.cjs')
if ($LASTEXITCODE -ne 0) { throw 'Demo regression failed' }
