$current = Get-Location
if ($IsWindows)
{
    $taget_dir = "build/win"
    New-Item -ItemType Directory $taget_dir -Force > $Null
    
    Set-Location $taget_dir
    cmake -G "Visual Studio 15 2017" -A x64 -T host=x64 `
          "${current}"
}
elseif ($IsLinux)
{
    $taget_dir = "build/linux"
    New-Item -ItemType Directory $taget_dir -Force > $Null
    
    Set-Location $taget_dir
    cmake "${current}"
}
elseif ($IsMacOS)
{
    $taget_dir = "build/osx"
    New-Item -ItemType Directory $taget_dir -Force > $Null
    
    Set-Location $taget_dir
    cmake "${current}"
}

cmake --build . --config Release
cmake --build . --config Debug

Set-Location $current