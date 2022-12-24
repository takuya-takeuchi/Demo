$current = Get-Location
$root = Split-Path $current -Parent
$eigen_source_dir = Join-Path ${root} eigen

$programName = "Reshape"

# build program
if ($global:IsWindows)
{
    $taget_dir = "build_win"
    New-Item -ItemType Directory $taget_dir -Force > $Null

    $install_dir = Join-Path $current "win"
    New-Item -ItemType Directory $install_dir -Force > $Null
    
    Set-Location $taget_dir
    $env:Eigen3_DIR = ${eigen_source_dir}
    cmake -G "Visual Studio 17 2022" -A x64 -T host=x64 `
          -D CMAKE_INSTALL_PREFIX="${install_dir}" `
          ..
}
elseif ($global:IsLinux)
{
    $taget_dir = "build_linux"
    New-Item -ItemType Directory $taget_dir -Force > $Null

    $install_dir = Join-Path $current "linux"
    New-Item -ItemType Directory $install_dir -Force > $Null
    
    Set-Location $taget_dir
    $env:Eigen3_DIR = "${eigen_source_dir}"
    cmake -D CMAKE_INSTALL_PREFIX="${install_dir}" `
          ..
}
elseif ($global:IsMacOS)
{
    $taget_dir = "build_osx"
    New-Item -ItemType Directory $taget_dir -Force > $Null

    $install_dir = Join-Path $current "osx"
    New-Item -ItemType Directory $install_dir -Force > $Null
    
    Set-Location $taget_dir
    $env:Eigen3_DIR = "${eigen_source_dir}"
    cmake -D CMAKE_INSTALL_PREFIX="${install_dir}" `
          ..
}

cmake --build . --config Release --target install

# run program
if ($global:IsWindows)
{
    $exe = Join-Path "${install_dir}" bin | `
           Join-Path -ChildPath "${programName}.exe"
    & "${exe}"
}
elseif ($global:IsLinux)
{
    $exe = Join-Path "${install_dir}" bin | `
           Join-Path -ChildPath "${programName}"
    & "${exe}"
}
elseif ($global:IsMacOS)
{
    $exe = Join-Path "${install_dir}" bin | `
           Join-Path -ChildPath "${programName}"
    & "${exe}"
}

Set-Location $current