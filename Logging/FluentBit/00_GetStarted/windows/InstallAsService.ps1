$current = $PSScriptRoot

$configPath = Join-Path $current "config.json"
$config = Get-Content -Path "${configPath}" | ConvertFrom-Json

$target = $config.target
$version = $config.version
$filename = $config.filename
$url = $config.url
$serviceName = $config.serviceName

$packageDir = Join-Path $current $target | `
             Join-Path -ChildPath $version

if (!(Test-Path("${packageDir}")))
{
    Write-Host "Please invoke 'Download.ps1' before install ${target}" -ForegroundColor Red
    exit
}

if (!([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole("Administrators"))
{
    Start-Process pwsh.exe "-File `"$PSCommandPath`"" -Verb RunAs;
    exit
}

$binary = Join-Path $packageDir bin | `
          Join-Path -ChildPath "fluent-bit.exe"
$config = Join-Path $packageDir conf | `
          Join-Path -ChildPath "fluent-bit.conf"
Write-Host "Install ${target} as Windows Service..." -ForegroundColor Blue
New-Service "${serviceName}" -BinaryPathName "`"${binary}`" -c `"${config}`"" -StartupType Automatic