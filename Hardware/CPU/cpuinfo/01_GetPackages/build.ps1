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
   $Configuration,

   [Parameter(
   Mandatory=$False,
   Position = 4
   )][string]
   $Device
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
   "android",
   "uwp"
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
$targetInstallRootDir = Join-Path $rootDir install
$targetInstallArchDir = Join-Path $targetInstallRootDir $os | `
                        Join-Path -ChildPath $architecture
$targetInstallDir = Join-Path $targetInstallArchDir share | `
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
    { "iphoneos", "iphonesimulator" -contains $_ }
    {
        $targetName = "hardware-cpu-cpuinfo-01"
        $project = Join-Path $current "xcode" | `
                   Join-Path -ChildPath "${targetName}.xcodeproj"
        $framework = Join-Path $current "xcode" | `
                     Join-Path -ChildPath "Frameworks"
        $xcframework = Join-Path $targetInstallRootDir "${buildTarget}.xcframework"
        Copy-Item "${xcframework}" "${framework}" -Recurse

        # deploy to booted simulator
        if ($os -eq "iphonesimulator")
        {
            open -a "Simulator"
        }

        # $Device = "platform=iOS Simulator,name=iPhone 14 Pro,OS=16.1"
        # $Device ="platform=iOS,id=9a1e9bf15489282f50f795bd0768752f28d62604"
        # clea and build
        xcodebuild clean -project "${project}"
        xcodebuild -project "${project}" `
                   -target "${targetName}" `
                   -sdk $os `
                   -configuration ${configuration} build `
                   -destination "${Device}" `
                   CONFIGURATION_BUILD_DIR="${installDir}" `
                   build

        # deploy to booted simulator
        if ($os -eq "iphonesimulator")
        {
            $app = Join-Path $installDir "${targetName}.app"
            xcrun simctl install Booted "${app}"   
        }          
    }
    "android"
    {
    }
    "uwp"
    {
        cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
              -D CMAKE_PREFIX_PATH="${targetInstallDir}" `
              -D CMAKE_BUILD_TYPE=${configuration} `
              -D TARGET_ARCHITECTURES="${architecture}" `
              -D CMAKE_SYSTEM_NAME=WindowsStore `
              -D CMAKE_SYSTEM_VERSION="10.0.18362" `
              =D CMAKE_VS_GLOBALS="WindowsTargetPlatformVersion=10.0.18362;WindowsTargetPlatformMinVersion=10.0.18362" `
              -D WINAPI_FAMILY=WINAPI_FAMILY_APP `
              -D _WINDLL=OFF `
              -D _WIN32_UNIVERSAL_APP=ON `
              $sourceDir
        cmake --build . --config ${configuration} --target install
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