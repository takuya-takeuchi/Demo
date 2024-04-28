$current = $PSScriptRoot

$configPath = Join-Path $current "config.json"
$config = Get-Content -Path "${configPath}" | ConvertFrom-Json

$serviceName = $config.serviceName

if (!([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole("Administrators"))
{
    Start-Process pwsh.exe "-File `"$PSCommandPath`"" -Verb RunAs;
    exit
}

Write-Host "Uninstall ${target}..." -ForegroundColor Blue
Remove-Service -Name ${serviceName}