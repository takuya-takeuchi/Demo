$current = $PSScriptRoot

$rootDir = Split-Path $current -Parent
$nginxDir = Join-Path $current nginx | `
            Join-Path -ChildPath sbin
$dstHtmlDir = Join-Path $current nginx | `
              Join-Path -ChildPath html
$srcHtmlDir = Join-Path $current html
if ($global:IsWindows)
{
}
elseif ($global:IsMacOS)
{
    # install-wasm/osx/opencv4/static/bin/opencv.js
    $opencvJs = Join-Path $rootDir install-wasm | `
                Join-Path -ChildPath osx | `
                Join-Path -ChildPath opencv4 | `
                Join-Path -ChildPath static | `
                Join-Path -ChildPath bin | `
                Join-Path -ChildPath opencv.js
    # copy opencv.js and contents
    Copy-Item "${opencvJs}" $dstHtmlDir -Force | Out-Null
    Copy-Item "${srcHtmlDir}/*" $dstHtmlDir -Force -Recurse | Out-Null

    $nginx = Join-Path $nginxDir nginx
    & "${nginx}" -s stop
    & "${nginx}"
}
elseif ($global:IsLinux)
{
}