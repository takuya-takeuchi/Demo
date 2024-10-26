$current = $PSScriptRoot

$emsdkVersion = "3.1.57"

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

$target = "libsodium"
$rootDir = Split-Path $current -Parent
$dstHtmlDir = Join-Path $current nginx | `
              Join-Path -ChildPath html
$srcHtmlDir = Join-Path $current html
$dstConfDir = Join-Path $current nginx | `
              Join-Path -ChildPath conf
$srcConfDir = Join-Path $current conf

$nginxDir = Join-Path $current nginx
$conf = Join-Path $srcConfDir nginx.conf
$wasm = Join-Path $rootDir install | `
        Join-Path -ChildPath $emsdkVersion | `
        Join-Path -ChildPath $os | `
        Join-Path -ChildPath ${target} | `
        Join-Path -ChildPath lib | `
        Join-Path -ChildPath "${target}.js"

# copy wasm, nginx.conf and contents
Copy-Item "${conf}" $dstConfDir -Force | Out-Null
Copy-Item "${wasm}" $dstHtmlDir -Force | Out-Null
Copy-Item "${srcHtmlDir}/*" $dstHtmlDir -Force -Recurse | Out-Null

Push-Location $nginxDir | Out-Null
if ($global:IsWindows)
{
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