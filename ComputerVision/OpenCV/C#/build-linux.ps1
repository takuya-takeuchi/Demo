#***************************************
#Arguments
#%1: Build Distribution (Release/Debug)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Distribution
)

$current = $PSScriptRoot

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

$target = "opencv4"
$version = "4.9.0"
$shared = "static"
$sharedFlag = "OFF"
$tagPostfix = "with-gstreamer"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $shared
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $shared |
              Join-Path -ChildPath ${Distribution}
$targetDir = Join-Path $installDir $target | `
             Join-Path -ChildPath lib | `
             Join-Path -ChildPath cmake

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsLinux)
{
    $path = Join-Path $current docker | `
            Join-Path -ChildPath ${Distribution}-dotnet6-opencv
    if (!(Test-Path(${path})))
    {
        Write-Host "${path} is missing" -ForegroundColor Red
        exit
    }

    docker build -t ${Distribution}-dotnet6-opencv-${tagPostfix} ${path} --build-arg OPENCV_VERSION="${version}"
    docker run -d --name ${Distribution}-dotnet6-opencv-${tagPostfix}-tmp -t ${Distribution}-dotnet6-opencv-${tagPostfix}
    docker cp ${Distribution}-dotnet6-opencv-${tagPostfix}-tmp:/usr/lib/libOpenCvSharpExtern.so ${installDir}
    docker stop ${Distribution}-dotnet6-opencv-${tagPostfix}-tmp
    docker rm ${Distribution}-dotnet6-opencv-${tagPostfix}-tmp

    # dotnet build -c Release
}
Pop-Location