$current = $PSScriptRoot

$target = "Open3D"
$version = "0.18.0"

# get os name
if ($global:IsWindows)
{
    $os = "win"
    $url = "https://github.com/isl-org/Open3D/releases/download/v${version}/open3d-devel-windows-amd64-${version}.zip"
    $basename = "open3d-devel-windows-amd64-${version}"
    $file = "${basename}.zip"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
    $url = "https://github.com/isl-org/Open3D/releases/download/v${version}/open3d-devel-darwin-arm64-${version}.tar.xz"
    $basename = "open3d-devel-darwin-arm64-${version}"
    $file = "${basename}.tar.xz"
}
elseif ($global:IsLinux)
{
    $os = "linux"
    $url = "https://github.com/isl-org/Open3D/releases/download/v${version}/open3d-devel-linux-x86_64-cxx11-abi-${version}.tar.xz"
    $basename = "open3d-devel-linux-x86_64-cxx11-abi-${version}"
    $file = "${basename}.tar.xz"
}

$installBaseDir = Join-Path $current install | `
                  Join-Path -ChildPath $target | `
                  Join-Path -ChildPath $os
New-Item -Type Directory ${installBaseDir} -Force | Out-Null

$path = Join-Path $current $file
$exist = Test-Path(${path})
if (!$exist)
{
    Write-Host "Download ${file} from ${url}" -ForegroundColor Blue
    Invoke-WebRequest "${url}" -OutFile "${file}"
}

$installDir = Join-Path $installBaseDir $version
if (Test-Path(${installDir}))
{
    Remove-Item $installDir -Force -Recurse | Out-Null
}

tar -xvf "${path}" -C "${installBaseDir}"

Move-Item "${installBaseDir}/${basename}" "${installDir}" -Force | Out-Null