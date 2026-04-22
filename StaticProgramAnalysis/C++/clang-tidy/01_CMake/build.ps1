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

$ConfigurationArray =
@(
   "Debug",
   "Release",
   "RelWithDebInfo",
   "MinSizeRel"
)

if ($ConfigurationArray.Contains($Configuration) -eq $False)
{
   $candidate = $ConfigurationArray -join "/"
   Write-Host "Specify build configuration [${candidate}]" -ForegroundColor Red
   exit -1
}

$current = $PSScriptRoot
$rootDir = Split-Path $current -Parent
$copnfigPath = Join-Path $current "build-config.json"
if (!(Test-Path($copnfigPath)))
{
    Write-Host "${copnfigPath} is missing" -ForegroundColor Red
    exit
}

$config = Get-Content -Path $copnfigPath | ConvertFrom-Json
$opencvVersion = $config.opencv.version
if ($config.opencv.shared)
{
    $opencvShared = "dynamic"
}
else
{
    $opencvShared = "static"
}

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

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $Configuration
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $Configuration

$OPENCV_INSTALL_DIR = Join-Path $current install | `
                      Join-Path -ChildPath $os | `
                      Join-Path -ChildPath opencv | `
                      Join-Path -ChildPath $opencvVersion | `
                      Join-Path -ChildPath $opencvShared | `
                      Join-Path -ChildPath $Configuration
if ($global:IsWindows)
{
    $OPENCV_CMAKE_DIR = Join-Path $OPENCV_INSTALL_DIR x64 | `
                        Join-Path -ChildPath vc17 | `
                        Join-Path -ChildPath staticlib
    # $OPENCV_CMAKE_DIR = Join-Path $OPENCV_INSTALL_DIR x64 | `
                        # Join-Path -ChildPath vc17 | `
    #                     Join-Path -ChildPath lib
}
else
{
    $OPENCV_CMAKE_DIR = Join-Path $OPENCV_INSTALL_DIR lib | `
                        Join-Path -ChildPath cmake | `
                        Join-Path -ChildPath opencv
}

$paths = @(
    "${OPENCV_CMAKE_DIR}"
)
foreach ($path in $paths)
{
    if (!(Test-Path($path)))
    {
        Write-Host "${path} is missing" -ForegroundColor Red
        exit
    }        
}

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir

$cmakeArgs = @()
if ($global:IsWindows)
{
    function CallVisualStudioDeveloperConsole()
    {
        $vs = "C:\Program Files\Microsoft Visual Studio\2022"
        $path = "${vs}\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
        if (!(Test-Path($path)))
        {
            $path = "${vs}\Professional\VC\Auxiliary\Build\vcvars64.bat"
        }
        if (!(Test-Path($path)))
        {
            $path = "${vs}\Community\VC\Auxiliary\Build\vcvars64.bat"
        }

        Write-Host "Use: ${path}" -ForegroundColor Green

        cmd.exe /c "call `"${path}`" && set > %temp%\vcvars.txt"
        Get-Content "${env:temp}\vcvars.txt" | Foreach-Object {
            if ($_ -match "^(.*?)=(.*)$") {
                Set-Content "env:\$($matches[1])" $matches[2]
            }
        }
    }
    CallVisualStudioDeveloperConsole

    if ($config.windows.msvcStaticRuntime)
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    }
    else
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL"
    }

    $cmakeArgs += @(
        "-G", "Visual Studio 17 2022", "-A", "x64", "-T", "host=x64"
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}"
    )
}
elseif ($global:IsMacOS)
{
    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
    )
}
elseif ($global:IsLinux)
{
    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
    )
}

$cmakeArgs += @(
    "-D CMAKE_PREFIX_PATH=${OPENCV_INSTALL_DIR}"
    "-D OpenCV_DIR=${OPENCV_CMAKE_DIR}"
    "${sourceDir}"
)

$configLogFile = Join-Path $buildDir cmake-config.log
$buildLogFile = Join-Path $buildDir cmake-build.log

cmake @cmakeArgs 2>&1 | Tee-Object -FilePath $configLogFile
cmake --build . --config ${Configuration} --target install 2>&1 | Tee-Object -FilePath $buildLogFile

Pop-Location