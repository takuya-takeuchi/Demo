$target = "fluentbit"
$version = "3.0.2"
$filename = "fluent-bit-${version}-win64.zip"
$url = "https://packages.fluentbit.io/windows/${filename}"
$serviceName = "fluent-bit"

if (!([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole("Administrators"))
{
    Start-Process pwsh.exe "-File `"$PSCommandPath`"" -Verb RunAs;
    exit
}

Write-Host "Uninstall ${target}..." -ForegroundColor Blue
Remove-Service -Name ${serviceName}