$target = "fluentbit"
$version = "3.0.2"
$filename = "fluent-bit-${version}-win64.zip"
$url = "https://packages.fluentbit.io/windows/${filename}"
$serviceName = "fluent-bit"

$current = $PSScriptRoot

$packageDir = Join-Path $current $target | `
             Join-Path -ChildPath $version

if (!(Test-Path("${packageDir}")))
{
    Write-Host "Please invoke `Download.ps1` before install ${target}" -ForegroundColor Red
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