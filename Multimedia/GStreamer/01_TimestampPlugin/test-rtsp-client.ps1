#***************************************
#Arguments
#%1: Build Configuration (Release/Debug)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Configuration
)

$current = $PSScriptRoot
$rootDir = Split-Path $current -Parent
$copnfigPath = Join-Path $rootDir "build-config.json"
if (!(Test-Path($copnfigPath)))
{
    Write-Host "${copnfigPath} is missing" -ForegroundColor Red
    exit
}

$config = Get-Content -Path $copnfigPath | ConvertFrom-Json

# get os name
if ($global:IsWindows)
{
    $os = "win"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
}
elseif ($global:IsLinux)
{
    $os = "linux"
}

$target = "gstreamer"
$version = $config.gstreamer.version

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program | `
            Join-Path -ChildPath $Configuration
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os
$targetInstallDir = Join-Path $rootDir install | `
                    Join-Path -ChildPath $os | `
                    Join-Path -ChildPath $target | `
                    Join-Path -ChildPath $version | `
                    Join-Path -ChildPath $Configuration
if (!(Test-Path(${targetInstallDir})))
{
    Write-Host "[Error] ${targetInstallDir} is missing" -ForegroundColor Red
    return
}
$targetBuildDir = Join-Path $rootDir build | `
                  Join-Path -ChildPath $os | `
                  Join-Path -ChildPath $target | `
                  Join-Path -ChildPath $version | `
                  Join-Path -ChildPath $Configuration | `
                  Join-Path -ChildPath subprojects | `
                  Join-Path -ChildPath gst-rtsp-server | `
                  Join-Path -ChildPath examples
if (!(Test-Path(${targetBuildDir})))
{
    Write-Host "[Error] ${targetBuildDir} is missing" -ForegroundColor Red
    return
}

if ($global:IsWindows)
{
    $installBinaryDir = Join-Path $installDir bin
    $gstLaunch = Join-Path $targetInstallDir bin | Join-Path -ChildPath gst-launch-1.0.exe
}
elseif ($global:IsMacOS)
{
    $installBinaryDir = Join-Path $installDir lib
    $gstLaunch = Join-Path $targetInstallDir bin | Join-Path -ChildPath gst-launch-1.0
}
elseif ($global:IsLinux)
{
    $installBinaryDir = Join-Path $installDir lib
    $env:GST_PLUGIN_SCANNER="../install/linux/gstreamer/${version}/Release/libexec/gstreamer-1.0/gst-plugin-scanner"
    $env:GST_PLUGIN_SYSTEM_PATH="../install/linux/gstreamer/${version}/Release/lib/x86_64-linux-gnu/gstreamer-1.0"
    $env:LD_LIBRARY_PATH="../install/linux/gstreamer/${version}/Release/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}"
    $gstLaunch = Join-Path $targetInstallDir bin | Join-Path -ChildPath gst-launch-1.0
}

$env:GST_DEBUG_DUMP_DOT_DIR=Join-Path $current dot
$env:GST_PLUGIN_PATH="${installBinaryDir}"
& $gstLaunch -e `
  filesrc location=input.mp4 ! `
  qtdemux name=d `
  d.video_0 ! queue ! `
  h264parse ! `
  rtspclientsink location=rtsp://192.168.11.102:8554/test