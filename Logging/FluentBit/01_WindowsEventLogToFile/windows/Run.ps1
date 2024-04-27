$current = $PSScriptRoot
$os = Split-Path $current -Leaf
$currentProject = Split-Path $current -Parent
$root = Split-Path $currentProject -Parent
$base = Join-Path $root "00_GetStarted" | Join-Path -ChildPath $os

$configPath = Join-Path $base "config.json"
$config = Get-Content -Path "${configPath}" | ConvertFrom-Json

$target = $config.target
$version = $config.version
$filename = $config.filename
$url = $config.url
$serviceName = $config.serviceName

$packageDir = Join-Path $base $target | `
             Join-Path -ChildPath $version

if (!(Test-Path("${packageDir}")))
{
    $scriptPath = Join-Path $base Download.ps1
    Write-Host "Please invoke '${scriptPath}' before run this script" -ForegroundColor Red
    exit
}

$binary = Join-Path $packageDir bin | `
          Join-Path -ChildPath "fluent-bit.exe"
$config = Join-Path $current "fluent-bit.conf"

Write-Host "Creating output directory for fluentbit plugins" -ForegroundColor Blue
New-Item -Type Directory -Force "C:\fluentbit\winevtlog" | Out-Null

Write-Host "Starting ${target}..." -ForegroundColor Blue
& "${binary}" -c "${config}"