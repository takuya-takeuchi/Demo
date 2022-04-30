$current = Get-Location

$boost_version = "1.73.0"
$boost_version2 = "1_73_0"
$boost_source_dir = "boost_${boost_version2}"

if ($IsWindows)
{
    $boost_dir = Join-Path $current "boost_win"

    $taget_dir = "build_win"
    New-Item -ItemType Directory $taget_dir -Force > $Null
    
    Set-Location $taget_dir
    $env:BOOST_DIR = "${boost_dir}/${boost_source_dir}/win64/cmake/boost-${boost_version}"
    cmake -G "Visual Studio 15 2017" -A x64 -T host=x64 `
          -D BOOST_DIR="$env:BOOST_DIR" `
          ..
}
elseif ($IsLinux)
{
    $boost_dir = Join-Path $current "boost_linux"

    $taget_dir = "build_linux"
    New-Item -ItemType Directory $taget_dir -Force > $Null
    
    Set-Location $taget_dir
    $env:Boost_DIR = "${boost_dir}/${boost_source_dir}/linux/cmake/Boost-${boost_version}"
    cmake -D Boost_DIR="$env:Boost_DIR" `
          ..
}
elseif ($IsMacOS)
{
    $boost_dir = Join-Path $current "boost_osx"

    $taget_dir = "build_osx"
    New-Item -ItemType Directory $taget_dir -Force > $Null
    
    Set-Location $taget_dir
    $env:BOOST_DIR = "${boost_dir}/${boost_source_dir}/osx/cmake/boost-${boost_version}"
    cmake -D BOOST_DIR="$env:BOOST_DIR" `
          ..
}

cmake --build . --config Release
cmake --build . --config Debug

Set-Location $current