#***************************************
#Arguments
#%1: Build Target (win/linux/osx/iphoneos/iphonesimulator/android)
#%2: Architecture (x86_64/arm64)
#%3: Build Configuration (Release/Debug/RelWithDebInfo/MinSizeRel)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Target,

   [Parameter(
   Mandatory=$True,
   Position = 2
   )][string]
   $Architecture,

   [Parameter(
   Mandatory=$True,
   Position = 3
   )][string]
   $Configuration
)

$os = $Target
$configuration = $Configuration
$architecture = $Architecture

$TargetArray =
@(
   "win",
   "linux",
   "osx",
   "iphoneos",
   "iphonesimulator",
   "android"
)

$ConfigurationArray =
@(
   "Debug",
   "Release",
   "RelWithDebInfo",
   "MinSizeRel"
)

$ArchitectureArray =
@(
   "arm64",
   "x86_64"
)

if ($TargetArray.Contains($os) -eq $False)
{
   $candidate = $TargetArray.Keys -join "/"
   Write-Host "Error: Specify Target [${candidate}]" -ForegroundColor Red
   exit -1
}

if ($ConfigurationArray.Contains($configuration) -eq $False)
{
   $candidate = $ConfigurationArray.Keys -join "/"
   Write-Host "Error: Specify Configuration [${candidate}]" -ForegroundColor Red
   exit -1
}

if ($ArchitectureArray.Contains($architecture) -eq $False)
{
   $candidate = $ArchitectureArray.Keys -join "/"
   Write-Host "Error: Specify Architecture [${candidate}]" -ForegroundColor Red
   exit -1
}

$current = $PSScriptRoot

$buildTarget = "cpuinfo"
$macosDeplolymentTarget = "11.0"

$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath program | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $architecture
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $architecture

$rootDir = Split-Path $current -Parent
$targetInstallRootDir = Join-Path $rootDir install | `
                        Join-Path -ChildPath $os | `
                        Join-Path -ChildPath $architecture
$targetInstallDir = Join-Path $targetInstallRootDir share | `
                    Join-Path -ChildPath $buildTarget

if (!(Test-Path(${targetInstallDir})))
{
    Write-Host "${targetInstallDir} is missing" -ForegroundColor Red
    exit
}

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir

switch ($os)
{
    "win"
    {
        cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
              -D CMAKE_PREFIX_PATH="${targetInstallDir}" `
              -D CMAKE_BUILD_TYPE=${configuration} `
              -D TARGET_ARCHITECTURES="$architecture" `
              $sourceDir
        cmake --build . --config ${configuration} --target install
    }
    "linux"
    {
        cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
              -D CMAKE_PREFIX_PATH="${targetInstallDir}" `
              -D CMAKE_BUILD_TYPE=${configuration} `
              -D TARGET_ARCHITECTURES="$architecture" `
              $sourceDir
        cmake --build . --config ${configuration} --target install
    }
    "osx"
    {
        cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
              -D CMAKE_PREFIX_PATH="${targetInstallDir}" `
              -D CMAKE_BUILD_TYPE=${configuration} `
              -D MACOSX_DEPLOYMENT_TARGET="${macosDeplolymentTarget}" 
              -D CMAKE_OSX_ARCHITECTURES="$architecture" `
              -D TARGET_ARCHITECTURES="$architecture" `
              $sourceDir
        cmake --build . --config ${configuration} --target install
    }
    "iphoneos"
    {
        $toolchain = Join-Path $rootDir "ios-cmake" | `
                     Join-Path -ChildPath "ios.toolchain.cmake"
        cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
              -D CMAKE_PREFIX_PATH="${targetInstallDir}" `
              -D CMAKE_BUILD_TYPE=${configuration} `
              -D MACOSX_DEPLOYMENT_TARGET="${macosDeplolymentTarget}" `
              -D PLATFORM=OS64 `
              -D CMAKE_TOOLCHAIN_FILE="${toolchain}" `
              -D TARGET_ARCHITECTURES="$architecture" `
              $sourceDir
        cmake --build . --config ${configuration} --target install
    }
    "iphonesimulator"
    {
        $toolchain = Join-Path $rootDir "ios-cmake" | `
                     Join-Path -ChildPath "ios.toolchain.cmake"
        cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
              -D CMAKE_PREFIX_PATH="${targetInstallDir}" `
              -D CMAKE_BUILD_TYPE=${configuration} `
              -D MACOSX_DEPLOYMENT_TARGET="${macosDeplolymentTarget}" `
              -D PLATFORM=SIMULATOR64 `
              -D CMAKE_TOOLCHAIN_FILE="${toolchain}" `
              -D TARGET_ARCHITECTURES="$architecture" `
              $sourceDir
        cmake --build . --config ${configuration} --target install
    }
    "android"
    {
    }
}
Pop-Location

# run
switch ($os)
{
    "win"
    {
        $programDir = Join-Path $installDir bin
        $program = Join-Path $programDir cpu-info.exe
        & ${program}
    }
    "linux"
    {
        $programDir = Join-Path $installDir bin
        $program = Join-Path $programDir cpu-info
        & ${program}
    }
    "osx"
    {
        $programDir = Join-Path $installDir bin
        $program = Join-Path $programDir cpu-info
        & ${program}
    }
}