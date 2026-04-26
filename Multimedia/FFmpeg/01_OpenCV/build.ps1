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
$copnfigPath = Join-Path $current "build-config.json"
if (!(Test-Path($copnfigPath)))
{
    Write-Host "${copnfigPath} is missing" -ForegroundColor Red
    exit
}

$config = Get-Content -Path $copnfigPath | ConvertFrom-Json
$openCvVersion = $config.opencv.version
if ($config.opencv.shared)
{
    $openCvShared = "dynamic"
    $openCvSharedFlag = "ON"
}
else
{
    $openCvShared = "static"
    $openCvSharedFlag = "OFF"
}

$current = $PSScriptRoot
$rootDir = Split-Path $current -Parent
$copnfigPath = Join-Path $rootDir "build-config.json"
if (!(Test-Path($copnfigPath)))
{
    Write-Host "${copnfigPath} is missing" -ForegroundColor Red
    exit
}
$config = Get-Content -Path $copnfigPath | ConvertFrom-Json
$ffmpegVersion = $config.ffmpeg.version
if ($config.ffmpeg.shared)
{
    $ffmpegShared = "dynamic"
    $ffmpegSharedFlag = "ON"
}
else
{
    $ffmpegShared = "static"
    $ffmpegSharedFlag = "OFF"
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
            Join-Path -ChildPath $version | `
            Join-Path -ChildPath $Configuration
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $version | `
              Join-Path -ChildPath $shared | `
              Join-Path -ChildPath $Configuration

$OPENCV_INSTALL_DIR = Join-Path $current install | `
                      Join-Path -ChildPath $os | `
                      Join-Path -ChildPath opencv | `
                      Join-Path -ChildPath $openCvVersion | `
                      Join-Path -ChildPath $openCvShared | `
                      Join-Path -ChildPath $Configuration
$OPENCV_CMAKECONFIG_FILE = (Get-ChildItem -Path $OPENCV_INSTALL_DIR -Recurse -File | Where-Object { $_.Name -eq "OpenCVModules.cmake" } | Select-Object -First 1).FullName
$OPENCV_CMAKE_DIR = Split-Path -Parent $OPENCV_CMAKECONFIG_FILE
$FFMPEG_INSTALL_DIR = Join-Path $rootDir install | `
                      Join-Path -ChildPath $os | `
                      Join-Path -ChildPath ffmpeg | `
                      Join-Path -ChildPath $ffmpegVersion | `
                      Join-Path -ChildPath $ffmpegShared | `
                      Join-Path -ChildPath $Configuration
$FFMPEG_PKGCONFIG_DIR = Join-Path $FFMPEG_INSTALL_DIR lib | `
                        Join-Path -ChildPath pkgconfig
$OPENH264_INSTALL_ROOT_DIR = Join-Path $rootDir install | `
                             Join-Path -ChildPath $os | `
                             Join-Path -ChildPath libopenh264
$OPENH264_PKGCONFIG_DIR = (Get-ChildItem -Path $OPENH264_INSTALL_ROOT_DIR -Recurse -Directory | Where-Object { $_.Name -eq "pkgconfig" } | Select-Object -First 1).FullName

$paths = @(
    "${OPENCV_CMAKE_DIR}"
    "${FFMPEG_PKGCONFIG_DIR}"
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

    $pkgConfigExe = Join-Path $current install | `
                    Join-Path -ChildPath $os | `
                    Join-Path -ChildPath pkg-config | `
                    Join-Path -ChildPath bin | `
                    Join-Path -ChildPath pkg-config.exe
    if (!(Test-Path(${pkgConfigExe})))
    {
        Write-Host "[Error] ${pkgConfigExe} is missing. Please run ./download-pkg-config.ps1" -ForegroundColor Red
        return
    }

    $cmakeArgs += @(
        "-G", "Visual Studio 17 2022", "-A", "x64", "-T", "host=x64"
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}"
        "-D PKG_CONFIG_EXECUTABLE=${pkgConfigExe}"
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
)

$cmakeArgs += @(
    "${sourceDir}"
)

$configLogFile = Join-Path $buildDir cmake-config.log
$buildLogFile = Join-Path $buildDir cmake-build.log

$env:PKG_CONFIG_PATH="${FFMPEG_PKGCONFIG_DIR}:${OPENH264_PKGCONFIG_DIR}"
cmake @cmakeArgs 2>&1 | Tee-Object -FilePath $configLogFile
$nproc = [Environment]::ProcessorCount
cmake --build . --config ${Configuration} --target install --parallel $nproc 2>&1 | Tee-Object -FilePath $buildLogFile

$FFMPEG_LIB_DIR = Join-Path $FFMPEG_INSTALL_DIR lib
$OPENH264_LIB_DIR = (Get-ChildItem -Path $OPENH264_INSTALL_ROOT_DIR -Recurse -Directory | Where-Object { $_.Name -eq "lib" } | Select-Object -First 1).FullName

$installDir = Join-Path $installDir bin
if ($global:IsWindows)
{
    $os = "win"
}
elseif ($global:IsMacOS)
{
    Get-ChildItem $FFMPEG_LIB_DIR -Recurse | 
        Where-Object { $_.Attributes -match "ReparsePoint" -and $_.Name -match "lib[^\.]+\.[0-9]+\.dylib" } |
        ForEach-Object { Copy-Item (Get-Item $_.FullName) (Join-Path $installDir $_.Name) -Force }
    Get-ChildItem $OPENH264_LIB_DIR -Recurse | 
        Where-Object { $_.Attributes -match "ReparsePoint" -and $_.Name -match "lib[^\.]+\.[0-9]+\.dylib" } |
        ForEach-Object { Copy-Item (Get-Item $_.FullName) (Join-Path $installDir $_.Name) -Force }
}
elseif ($global:IsLinux)
{
    Get-ChildItem $FFMPEG_LIB_DIR -Recurse | 
        Where-Object { $_.Attributes -match "ReparsePoint" -and $_.Name -match "lib.+.so.[0-9]+" } |
        ForEach-Object { Copy-Item (Get-Item $_.FullName) (Join-Path $installDir $_.Name) -Force }
    Get-ChildItem $OPENH264_LIB_DIR -Recurse | 
        Where-Object { $_.Attributes -match "ReparsePoint" -and $_.Name -match "lib.+.so.[0-9]+" } |
        ForEach-Object { Copy-Item (Get-Item $_.FullName) (Join-Path $installDir $_.Name) -Force }
}

Pop-Location