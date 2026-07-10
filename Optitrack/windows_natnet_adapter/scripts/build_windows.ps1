param(
    [string]$NatNetSdkDir = $env:NATNET_SDK_DIR,
    [string]$BuildDir = "build",
    [string]$Configuration = "Debug"
)

$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($NatNetSdkDir)) {
    throw "Set NATNET_SDK_DIR or pass -NatNetSdkDir C:\NatNetSDK"
}

if (-not (Test-Path $NatNetSdkDir)) {
    throw "NatNet SDK directory does not exist: $NatNetSdkDir"
}

$env:NATNET_SDK_DIR = $NatNetSdkDir

cmake -S . -B $BuildDir -G "Visual Studio 17 2022" -A x64 -DOPTITRACK_ADAPTER_WITH_NATNET=ON
cmake --build $BuildDir --config $Configuration

Write-Host "Build finished: $BuildDir\$Configuration\windows_natnet_adapter.exe"

