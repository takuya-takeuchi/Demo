#***************************************
#Arguments
#%1: Build Target iphoneos/iphonesimulator)
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
   "iphoneos",
   "iphonesimulator"
)

$ConfigurationArray =
@(
   "Debug",
   "Release"
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

$buildTarget = "ObjectiveLuhnWrapper"
$wrappedLibrary = "Luhn"

# build
$sourceDir = Join-Path $current ${buildTarget}
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $architecture | `
            Join-Path -ChildPath $buildTarget
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
            Join-Path -ChildPath $architecture | `
              Join-Path -ChildPath $buildTarget

$cmakeDir = Split-Path $current -Parent
$rootDir = Split-Path $cmakeDir -Parent
$toolchainDir = Join-Path $rootDir "toolchains"

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
$BUILD_SHARED_LIBS="ON"
$CMAKE_IOS_INSTALL_COMBINED="NO"

$toolchain = Join-Path $toolchainDir "${architecture}-ios.cmake"

switch ($os)
{
    "iphoneos"
    {
      $frameworkDir = Join-Path $current install | `
                      Join-Path -ChildPath "${wrappedLibrary}.xcframework" | `
                      Join-Path -ChildPath "ios-${architecture}"
    }
    "iphonesimulator"
    {
      $frameworkDir = Join-Path $current install | `
                      Join-Path -ChildPath "${wrappedLibrary}.xcframework" | `
                      Join-Path -ChildPath "ios-${architecture}-simulator"
    }
}

cmake -D CMAKE_BUILD_TYPE=$Configuration `
      -D CMAKE_INSTALL_PREFIX=${installDir} `
      -D BUILD_SHARED_LIBS="ON" `
      -D CMAKE_EXE_LINKER_FLAGS="-std=c++17 -stdlib=libc++" `
      -D CMAKE_IOS_INSTALL_COMBINED=$CMAKE_IOS_INSTALL_COMBINED `
      -D CMAKE_OSX_SDK="${os}" `
      -D CMAKE_OSX_DEPLOYMENT_TARGET="14.0" `
      -D CMAKE_TOOLCHAIN_FILE="${toolchain}" `
      -D LUHN_SDK_DIR="${frameworkDir}" `
      "${sourceDir}"
cmake --build . --config ${Configuration} --target install
Pop-Location