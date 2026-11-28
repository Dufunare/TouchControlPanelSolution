param([string]$Compiler = 'g++', [switch]$BuildOnly)
$ErrorActionPreference = 'Stop'
$outputDir = Join-Path $PSScriptRoot 'audit-build'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$executables = @()
foreach ($test in Get-ChildItem (Join-Path $PSScriptRoot '*_tests.cpp')) {
    $exe = Join-Path $outputDir ($test.BaseName + '.exe')
    $argsList = @('-std=c++17','-O0','-Wall','-Wextra',$test.FullName,'-o',$exe)
    if ($test.BaseName -eq 'bounded_socket_tests') { $argsList += '-lws2_32' }
    & $Compiler @argsList
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $($test.Name)" }
    $executables += $exe
}
$root = Split-Path -Parent $PSScriptRoot
$exe = Join-Path $outputDir 'communication_backend_integration.exe'
& $Compiler -std=c++17 -O0 -Wall -Wextra -DPCH_H -DNOMINMAX `
    "-I$root/ControlBackend" "-I$root/ControlBackend/include" "-I$root/shared" `
    (Join-Path $PSScriptRoot 'communication_backend_integration.cpp') `
    (Join-Path $root 'ControlBackend/src/CommunicationBackend.cpp') -lws2_32 -o $exe
if ($LASTEXITCODE -ne 0) { throw 'Communication backend compile failed' }
$executables += $exe
if ($BuildOnly) { exit 0 }
foreach ($test in $executables) {
    & $test
    if ($LASTEXITCODE -ne 0) { throw "Test failed: $test" }
}
Write-Output "PASS: $($executables.Count) test executables"
