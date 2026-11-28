param(
    [string]$QtRoot='C:\Qt\6.11.0\msvc2022_64',
    [string]$CMake='C:\Qt\Tools\CMake_64\bin\cmake.exe',
    [string]$VisualStudioRoot='C:\Program Files\Microsoft Visual Studio\18\Community'
)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$outputDir=Join-Path $root 'x64/QtWarningTests'
. (Join-Path $VisualStudioRoot 'Common7/Tools/Launch-VsDevShell.ps1') -Arch amd64 -HostArch amd64 -SkipAutomaticLocation -NoLogo
$env:QTFRAMEWORK_BYPASS_LICENSE_CHECK='1'
& $CMake -S (Join-Path $PSScriptRoot 'qt-video-smoke') -B $outputDir -G 'NMake Makefiles' "-DCMAKE_PREFIX_PATH=$QtRoot" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_DEBUG=/Od
if($LASTEXITCODE -ne 0){throw 'CMake configuration failed'}
& $CMake --build $outputDir --target camera_warning_smoke qt_control_compile qt_control_thread_smoke
if($LASTEXITCODE -ne 0){throw 'Qt compilation failed'}
$env:PATH="$QtRoot/bin;"+$env:PATH
$previousPlatform=$env:QT_QPA_PLATFORM
try {
    $env:QT_QPA_PLATFORM='offscreen'
    & (Join-Path $outputDir 'camera_warning_smoke.exe') (Join-Path $outputDir 'warning-preview.png')
    if($LASTEXITCODE -ne 0){throw 'Camera warning test failed'}
    & (Join-Path $outputDir 'qt_control_thread_smoke.exe')
    if($LASTEXITCODE -ne 0){throw 'Local relay thread test failed'}
} finally {$env:QT_QPA_PLATFORM=$previousPlatform}
