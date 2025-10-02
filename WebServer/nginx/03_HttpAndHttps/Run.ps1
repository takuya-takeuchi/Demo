$current = $PSScriptRoot

# get os name
if ($global:IsWindows)
{
    $os = "win"
    $nginx = "nginx.exe"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
    $nginx = "nginx"
}
elseif ($global:IsLinux)
{
    $os = "linux"
    $nginx = "nginx"
}

$nginxDir = Join-Path $current nginx

Push-Location $nginxDir | Out-Null
if ($global:IsWindows)
{
    if (!([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole("Administrators"))
    {
        Start-Process powershell.exe "-File `"$PSCommandPath`"" -Verb RunAs; exit
    }

    $nginx = Join-Path $nginxDir $nginx
    & "${nginx}" -s stop
    start "${nginx}"
}
else
{
    $nginx = Join-Path $nginxDir sbin | `
             Join-Path -ChildPath $nginx
    & "${nginx}" -s stop
    & "${nginx}"
}
Pop-Location