$current = $PSScriptRoot
$root = Split-Path $current -Parent

if ($global:IsWindows)
{
    $os = "windows"
}
elseif ($global:IsLinux)
{
    $os = "windows"
}
else
{
    Write-Host "[Error] This platform is not supported" -ForegroundColor Red
    exit
}

# Setup ROS2 packages
$branch = ""
$ros2Packages = @(
	"humble"
)
$installedRos2Dir = ""
foreach($ros2 in $ros2Packages)
{
    $branch = $ros2
    $installedRos2Dir = Join-Path $root "ros2" | Join-Path -ChildPath $ros2 | Join-Path -ChildPath "ros2-windows"
    if (Test-Path(${installedRos2Dir}))
    {
        break
    }
}

if (!(Test-Path($installedRos2Dir)))
{
    Write-Host "[Error] ROS2 package is missing. You must invoke Setup" -ForegroundColor Red
    exit
}

Write-Host "[Info] ROS2: ${installedRos2Dir}" -ForegroundColor Green

$target = Join-Path $root examples
Push-Location $target | Out-Null
git submodule update --init --recursive .
git fetch -ap
git checkout $branch

if ($global:IsWindows)
{
    # Check Visual Studio is installed or not
    $visualStudioShells = @(
        "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
        "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat"
        "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
        "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat"
        "C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\VC\Auxiliary\Build\vcvars64.bat"
        "C:\Program Files (x86)\Microsoft Visual Studio\2019\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
        "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvars64.bat"
        "C:\Program Files (x86)\Microsoft Visual Studio\2017\Professional\VC\Auxiliary\Build\vcvars64.bat"
        "C:\Program Files (x86)\Microsoft Visual Studio\2017\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
        "C:\Program Files (x86)\Microsoft Visual Studio\2015\Community\VC\Auxiliary\Build\vcvars64.bat"
        "C:\Program Files (x86)\Microsoft Visual Studio\2015\Professional\VC\Auxiliary\Build\vcvars64.bat"
        "C:\Program Files (x86)\Microsoft Visual Studio\2015\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
    )
    
    $visualStudioShell = ""
    foreach ($shell in $visualStudioShells)
    {
        $visualStudioShell = $shell
        if (Test-Path($visualStudioShell))
        {
            break
        }
    }
    
    if (!(Test-Path($visualStudioShell)))
    {
        Write-Host "[Error] Visual Studio is not installed or x64 Native Tools Command Prompt is missing" -ForegroundColor Red
        exit
    }

    function Call($batfile)
    {
        cmd.exe /c "call `"${batfile}`" && set > %temp%\vars.txt"
        Get-Content "${env:temp}\vars.txt" | Foreach-Object {
            if ($_ -match "^(.*?)=(.*)$") {
                Set-Content "env:\$($matches[1])" $matches[2]
            }
        }
    }
    
    $env:PATH = "C:\Python38;C:\Python38\Scripts;C:\Windows\System32\WindowsPowerShell\v1.0;C:\Program Files\CMake\bin;C:\Windows\System32;C:\Program Files\Cppcheck;"
    $env:PYTHONPATH = "C:\Python38\python.exe"
    $env:PYTHONHOME = "C:\Python38"
    $env:PATHEXT = $env:PATHEXT + ";.PY;.PYW"
    $env:COLCON_PYTHON_EXECUTABLE = $env:PYTHONPATH

    Call("${visualStudioShell}")

    $scriptPath = Join-Path $installedRos2Dir "local_setup.ps1"
    . "${scriptPath}"
}
elseif ($global:IsLinux)
{
}

# build
$testLog = Join-Path $current test.log
# `--event-handlers desktop_notification-` argument suppress the following error message
# WNDPROC return value cannot be converted to LRESULT
# TypeError: WPARAM is simple, so must be an int object (got NoneType)
colcon test --event-handlers desktop_notification- 2>&1 | Tee-Object -FilePath $testLog
# colcon test --event-handlers desktop_notification- --event-handlers console_direct+ 2>&1 | Tee-Object -FilePath $testLog
Pop-Location