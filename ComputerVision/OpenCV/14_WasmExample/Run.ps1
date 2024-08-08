$current = $PSScriptRoot

$emsdkVersion = "1.39.15"

$rootDir = Split-Path $current -Parent
$dstHtmlDir = Join-Path $current nginx | `
              Join-Path -ChildPath html
$srcHtmlDir = Join-Path $current html
$dstConfDir = Join-Path $current nginx | `
              Join-Path -ChildPath conf
$srcConfDir = Join-Path $current conf
if ($global:IsWindows)
{
    $nginxDir = Join-Path $current nginx
    $conf = Join-Path $srcConfDir nginx.conf
    $opencvJs = Join-Path $rootDir install-wasm | `
                Join-Path -ChildPath $emsdkVersion | `
                Join-Path -ChildPath win | `
                Join-Path -ChildPath opencv4 | `
                Join-Path -ChildPath static | `
                Join-Path -ChildPath bin | `
                Join-Path -ChildPath opencv.js

    # copy opencv.js, nginx.conf and contents
    Copy-Item "${conf}" $dstConfDir -Force | Out-Null
    Copy-Item "${opencvJs}" $dstHtmlDir -Force | Out-Null
    Copy-Item "${srcHtmlDir}/*" $dstHtmlDir -Force -Recurse | Out-Null

    $nginx = Join-Path $nginxDir "nginx.exe"
    Push-Location $nginxDir | Out-Null
    & "${nginx}" -s stop
    start "${nginx}"
    Pop-Location
}
elseif ($global:IsMacOS)
{
    $nginxDir = Join-Path $current nginx | `
                Join-Path -ChildPath sbin
    $conf = Join-Path $srcConfDir nginx.conf
    $opencvJs = Join-Path $rootDir install-wasm | `
                Join-Path -ChildPath $emsdkVersion | `
                Join-Path -ChildPath osx | `
                Join-Path -ChildPath opencv4 | `
                Join-Path -ChildPath static | `
                Join-Path -ChildPath bin | `
                Join-Path -ChildPath opencv.js

    # copy opencv.js, nginx.conf and contents
    Copy-Item "${conf}" $dstConfDir -Force | Out-Null
    Copy-Item "${opencvJs}" $dstHtmlDir -Force | Out-Null
    Copy-Item "${srcHtmlDir}/*" $dstHtmlDir -Force -Recurse | Out-Null

    $nginx = Join-Path $nginxDir nginx
    Push-Location $nginxDir | Out-Null
    & "${nginx}" -s stop
    & "${nginx}"
    Pop-Location
}
elseif ($global:IsLinux)
{
}