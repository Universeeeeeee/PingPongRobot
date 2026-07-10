param(
    [string]$Config = ".\config\lab_config.json",
    [string]$Executable = ".\build\Debug\windows_natnet_adapter.exe",
    [string]$Output = ".\logs\smoke_session.jsonl",
    [int]$Seconds = 10
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path $Config)) {
    throw "Config file does not exist: $Config. Copy config\example_config.json to config\lab_config.json first."
}

if (-not (Test-Path $Executable)) {
    throw "Executable does not exist: $Executable. Run scripts\build_windows.ps1 first."
}

New-Item -ItemType Directory -Force -Path (Split-Path $Output) | Out-Null

Write-Host "Running smoke test for $Seconds seconds..."
Write-Host "Config: $Config"
Write-Host "Output: $Output"

$process = Start-Process -FilePath $Executable -ArgumentList $Config -RedirectStandardOutput $Output -NoNewWindow -PassThru
Start-Sleep -Seconds $Seconds

if (-not $process.HasExited) {
    Stop-Process -Id $process.Id
}

python tools\replay_jsonl.py $Output

