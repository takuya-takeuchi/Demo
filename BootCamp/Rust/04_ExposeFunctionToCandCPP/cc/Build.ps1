$current = Get-Location
$root = Split-Path $current -Parent
$rust_lib_dir = Join-Path ${root} sample | `
                Join-Path -ChildPath target | `
                Join-Path -ChildPath release

$programName = "cc"

# build program
if ($global:IsWindows)
{
    $taget_dir = "build_win"
    New-Item -ItemType Directory $taget_dir -Force > $Null

    $install_dir = Join-Path $current "win"
    New-Item -ItemType Directory $install_dir -Force > $Null
    
    Set-Location $taget_dir
    cmake -G "Visual Studio 17 2022" -A x64 -T host=x64 `
          -D CMAKE_INSTALL_PREFIX="${install_dir}" `
          -D RUST_LIBS_DIR="${rust_lib_dir}" `
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

Set-Location $current