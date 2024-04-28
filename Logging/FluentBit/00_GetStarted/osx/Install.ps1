$current = $PSScriptRoot

$configPath = Join-Path $current "config.json"
$config = Get-Content -Path "${configPath}" | ConvertFrom-Json

$version = $config.version
$packageName = $config.packageName

Write-Host "Install ${target}..." -ForegroundColor Blue
& brew install ${packageName}