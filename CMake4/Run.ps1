$current = Get-Location

if ($IsWindows)
{
    $taget_dir = "build_win/Release"
}
elseif ($IsLinux)
{
    $taget_dir = "build_linux"
}
elseif ($IsMacOS)
{
    $taget_dir = "build_osx"
}

Set-Location $taget_dir

if ($IsWindows)
{
    cmd.exe /c CMakeBoost.exe test_dir
}
elseif ($IsLinux)
{
    ./CMakeBoost test_dir
}
elseif ($IsMacOS)
{
    ./CMakeBoost test_dir
}

Set-Location $current