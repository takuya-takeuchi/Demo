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
$rootDir = Split-Path $rootDir -Parent
$rootDir = Split-Path $rootDir -Parent
$configPath = Join-Path $rootDir "build-config.json"
if (!(Test-Path($configPath)))
{
    Write-Host "${configPath} is missing" -ForegroundColor Red
    exit
}

$config = Get-Content -Path $configPath | ConvertFrom-Json

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

$awsSdkCppVersion = $config.awssdkcpp.version
if ($config.awssdkcpp.shared)
{
    $awsSdkCppShared = "dynamic"
}
else
{
    $awsSdkCppShared = "static"
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

$AWSSDKCPP_INSTALL_DIR = Join-Path $rootDir install | `
                         Join-Path -ChildPath $os | `
                         Join-Path -ChildPath aws-sdk-cpp | `
                         Join-Path -ChildPath $awsSdkCppVersion | `
                         Join-Path -ChildPath $awsSdkCppShared | `
                         Join-Path -ChildPath $Configuration
$AWSSDKCPP_CMAKE_DIR = Join-Path $AWSSDKCPP_INSTALL_DIR lib | `
                       Join-Path -ChildPath cmake | `
                       Join-Path -ChildPath AWSSDK

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
    chcp 65001

    if ($config.windows.msvcStaticRuntime)
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded"
    }
    else
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL"
    }

    $CMAKE_PREFIX_PATH = $AWSSDKCPP_CMAKE_FILE_DIRS -Join ";"
    $cmakeArgs += @(
        "-G", "Visual Studio 17 2022", "-A", "x64", "-T", "host=x64"
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}"
        "-D CMAKE_PREFIX_PATH=${AWSSDKCPP_INSTALL_DIR}"
        "-D AWSSDK_DIR=${AWSSDKCPP_CMAKE_DIR}"
    )
}
elseif ($global:IsMacOS)
{
    $CMAKE_PREFIX_PATH = $AWSSDKCPP_CMAKE_FILE_DIRS -Join ":"
    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_PREFIX_PATH=${AWSSDKCPP_INSTALL_DIR}"
        "-D AWSSDK_DIR=${AWSSDKCPP_CMAKE_DIR}"
    )
}
elseif ($global:IsLinux)
{
    # $S2N_CMAKE_DIR = Join-Path $AWSSDKCPP_INSTALL_DIR lib | `
    #                    Join-Path -ChildPath s2n | `
    #                    Join-Path -ChildPath cmake
    
    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_PREFIX_PATH=${AWSSDKCPP_INSTALL_DIR}"
        "-D AWSSDK_DIR=${AWSSDKCPP_CMAKE_DIR}"
    )
}

$cmakeArgs += @(
    "${sourceDir}"
)

$configLogFile = Join-Path $buildDir cmake-config.log
$buildLogFile = Join-Path $buildDir cmake-build.log

cmake @cmakeArgs 2>&1 | Tee-Object -FilePath $configLogFile
$nproc = [Environment]::ProcessorCount
cmake --build . --config ${Configuration} --target install --parallel $nproc 2>&1 | Tee-Object -FilePath $buildLogFile

Pop-Location