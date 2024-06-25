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
$ros2Packages = @(
	"humble"
)
$installedRos2Dir = ""
foreach($ros2 in $ros2Packages)
{
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

if ($global:IsWindows)
{
    $env:PATH = "C:\Python38;C:\Python38\Scripts"
    $env:PYTHONPATH = "C:\Python38\python.exe"
    $env:PYTHONHOME = "C:\Python38"
    $env:PATHEXT = $env:PATHEXT + ";.PY;.PYW"
    $env:COLCON_PYTHON_EXECUTABLE = $env:PYTHONPATH

    $scriptPath = Join-Path $installedRos2Dir "local_setup.ps1"
    . "${scriptPath}"
}
elseif ($global:IsLinux)
{
}

ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function