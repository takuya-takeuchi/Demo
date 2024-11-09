#***************************************
#Arguments
#%1: Build Target (win/linux/osx/iphoneos/iphonesimulator/android/uwp)
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
        cmake -G "Visual Studio 17 2022" -A x64 -T host=x64 `
              -D CMAKE_INSTALL_PREFIX=${installDir} `
              -D CMAKE_PREFIX_PATH="${targetInstallDir}" `
              -D CMAKE_BUILD_TYPE=${configuration} `
              -D CMAKE_SYSTEM_NAME=WindowsStore `
              -D CMAKE_SYSTEM_VERSION="10.0.17134.0" `
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