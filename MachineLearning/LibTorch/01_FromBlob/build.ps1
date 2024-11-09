#***************************************
#Arguments
#%1: Build Target (win/linux/osx/ios/android)
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
   "ios",
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
   $candidate = $TargetArray -join "/"
   Write-Host "Error: Specify Target [${candidate}]" -ForegroundColor Red
   exit -1
}

if ($ConfigurationArray.Contains($configuration) -eq $False)
{
   $candidate = $ConfigurationArray -join "/"
   Write-Host "Error: Specify Configuration [${candidate}]" -ForegroundColor Red
   exit -1
}

if ($ArchitectureArray.Contains($architecture) -eq $False)
{
   $candidate = $ArchitectureArray -join "/"
   Write-Host "Error: Specify Architecture [${candidate}]" -ForegroundColor Red
   exit -1
}

$current = $PSScriptRoot

$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath program | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $architecture
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $architecture

$rootDir = Split-Path $current -Parent
$torchInstallRootDir = Join-Path $rootDir install | `
                       Join-Path -ChildPath $os | `
                       Join-Path -ChildPath $architecture
$torchInstallLibDir = Join-Path $torchInstallRootDir lib
$torchInstallDir = Join-Path $torchInstallRootDir share | `
                   Join-Path -ChildPath cmake | `
                   Join-Path -ChildPath Torch
$protobufLibInstallDir = Join-Path $torchInstallRootDir lib

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir

switch ($os)
{
    "win"
    {
        cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
              -D CMAKE_PREFIX_PATH="${torchInstallDir}" `
              -D Protobuf_LIBRARIES="${protobufLibInstallDir}/libprotobuf.lib" `
              -D TARGET_ARCHITECTURES="$architecture" `
              $sourceDir
        cmake --build . --config ${configuration} --target install
    }
    "linux"
    {
        cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
              -D CMAKE_PREFIX_PATH="${torchInstallDir}" `
              -D TORCH_ADDITIONAL_LIBS_DIRS="${torchInstallLibDir}"  `
              -D Protobuf_LIBRARIES="${protobufLibInstallDir}/libprotobuf.a" `
              -D Protobuf_LIBRARIES="${protobufLibInstallDir}/libsleef.a" `
              -D TARGET_ARCHITECTURES="$architecture" `
              $sourceDir
        cmake --build . --config ${configuration} --target install
    }
    "osx"
    {
        
        switch ($architecture)
        {
            "x86_64"
            {
                cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
                      -D CMAKE_PREFIX_PATH="${torchInstallDir}" `
                      -D TORCH_ADDITIONAL_LIBS_DIRS="${torchInstallLibDir}" `
                      -D Protobuf_LIBRARIES="${protobufLibInstallDir}/libprotobuf.a" `
                      -D Protobuf_LIBRARIES="${protobufLibInstallDir}/libsleef.a" `
                      -D MACOSX_DEPLOYMENT_TARGET="11.0" `
                      -D CMAKE_OSX_ARCHITECTURES="$architecture" `
                      -D TARGET_ARCHITECTURES="$architecture" `
                      $sourceDir
                cmake --build . --config ${configuration} --target install
            }
            "arm64"
            {
                cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
                      -D CMAKE_PREFIX_PATH="${torchInstallDir}" `
                      -D Protobuf_LIBRARIES="${protobufLibInstallDir}/libprotobuf.a" `
                      -D Protobuf_LIBRARIES="${protobufLibInstallDir}/libsleef.a" `
                      -D MACOSX_DEPLOYMENT_TARGET="11.0" `
                      -D CMAKE_OSX_ARCHITECTURES="$architecture" `
                      -D TARGET_ARCHITECTURES="$architecture" `
                      $sourceDir
                cmake --build . --config ${configuration} --target install
            }
        }
    }
    "ios"
    {
        cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
              -D CMAKE_PREFIX_PATH="${torchInstallDir}" `
              -D TORCH_ADDITIONAL_LIBS_DIRS="${torchInstallLibDir}" `
              -D Protobuf_LIBRARIES="${protobufLibInstallDir}/libprotobuf.a" `
              -D Protobuf_LIBRARIES="${protobufLibInstallDir}/libsleef.a" `
              -D MACOSX_DEPLOYMENT_TARGET="11.0" `
              -D CMAKE_OSX_ARCHITECTURES="$architecture" `
              -D CMAKE_SYSTEM_NAME="iOS" `
              -D CMAKE_OSX_SYSROOT="iphoneos" `
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
        $program = Join-Path $programDir Demo.exe
        & ${program}
    }
    "linux"
    {
        $programDir = Join-Path $installDir bin
        $program = Join-Path $programDir Demo
        & ${program}
    }
    "osx"
    {
        $programDir = Join-Path $installDir bin
        $program = Join-Path $programDir Demo
        & ${program}
    }
}