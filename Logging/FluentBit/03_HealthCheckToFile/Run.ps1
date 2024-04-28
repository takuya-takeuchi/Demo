$current = $PSScriptRoot
if ($global:IsMacOS)
{
    $os = "osx"
}
elseif ($global:IsLinux)
{
    $os = "linux"
}
else
{
    Write-Host "Error: This plaform is not support" -ForegroundColor Red
    exit -1
}
$root = Split-Path $current -Parent
$base = Join-Path $root "00_GetStarted" | Join-Path -ChildPath $os

$configPath = Join-Path $base "config.json"
$config = Get-Content -Path "${configPath}" | ConvertFrom-Json

$package = $config.serviceName

$config = Join-Path $current "fluent-bit-${os}.conf"

Write-Host "Creating output directory for fluentbit plugins" -ForegroundColor Blue
if ($global:IsMacOS)
{
    New-Item -Type Directory -Force "${current}/logs" | Out-Null
}
elseif ($global:IsLinux)
{
    New-Item -Type Directory -Force "${current}/logs" | Out-Null
}

Write-Host "Starting ${target}..." -ForegroundColor Blue
if ($global:IsMacOS)
{
    & "${package}"-c "${config}"
}
elseif ($global:IsLinux)
{
    & "/opt/fluent-bit/bin/fluent-bit"-c "${config}"
}