$current = $PSScriptRoot

$configPath = Join-Path $current "config.json"
$config = Get-Content -Path "${configPath}" | ConvertFrom-Json

$packageName = $config.packageName

Write-Host "Uninstall ${target}..." -ForegroundColor Blue
& brew uninstall ${packageName}