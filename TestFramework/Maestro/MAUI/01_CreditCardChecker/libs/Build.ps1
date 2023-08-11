#***************************************
#Arguments
#%1: Target (android/iphoneos/iphonesimulator)
#%2: Architecture (x86_64/arm64)
#%3: Build Configuration (Release/Debug)
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

# root config
$macosDeplolymentTarget = "11.0"
$androidNativeApiLevel = 21
$cmakeVersion = "3.22.1"

$target = $Target
$architecture = $Architecture
$configuration = $Configuration

$TargetArray =
@(
   "windows",
   "android",
   "iphoneos"
   "iphonesimulator"
)

$ArchitectureArray =
@(
   "arm64",
   "x86_64"
)

$ConfigurationArray =
@(
   "Debug",
   "Release"
)

if ($TargetArray.Contains($target) -eq $False)
{
   $candidate = $ConfigurationArray.Keys -join "/"
   Write-Host "Error: Specify Target [${candidate}]" -ForegroundColor Red
   exit -1
}

if ($ArchitectureArray.Contains($architecture) -eq $False)
{
   $candidate = $ArchitectureArray.Keys -join "/"
   Write-Host "Error: Specify Architecture [${candidate}]" -ForegroundColor Red
   exit -1
}

if ($ConfigurationArray.Contains($configuration) -eq $False)
{
   $candidate = $ConfigurationArray.Keys -join "/"
   Write-Host "Error: Specify Configuration [${candidate}]" -ForegroundColor Red
   exit -1
}

if ($target -eq "android")
{
   switch ($architecture)
   {
      "arm64"  { $architecture = "arm64-v8a" }
      "x86_64" { $architecture = "x86_64" }
   }
}

$current = $PSScriptRoot
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $architecture
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $target | `
            Join-Path -ChildPath $architecture

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir

if ($target -eq "windows")
{
   cmake -D CMAKE_BUILD_TYPE=${configuration} `
         -D CMAKE_INSTALL_PREFIX=${installDir} `
         $sourceDir
   cmake --build . --config ${configuration} --target install
}
elseif ($target -eq "android")
{
   $androidHome = $env:ANDROID_HOME
   if (!($androidHome))
   {
      Write-Host "ANDROID_HOME is missing" -ForegroundColor Red
      exit
   }

   if (!(Test-Path($androidHome)))
   {
      Write-Host "ANDROID_HOME: ${androidHome} does not exist" -ForegroundColor Red
      exit
   }

   $cmake = ""
   if ($IsWindows)
   {
      $cmake = "cmake.exe"
      $cmake = Join-Path $androidHome "cmake" | `
               Join-Path -ChildPath ${cmakeVersion} | `
               Join-Path -ChildPath "bin" | `
               Join-Path -ChildPath $cmake

      if (!(Test-Path($cmake)))
      {
         Write-Host "${cmake} does not exist" -ForegroundColor Red
         exit
      }
   }
   elseif ($IsLinux)
   {
      $cmake = "cmake"
      $cmake = Join-Path $androidHome "cmake" | `
               Join-Path -ChildPath ${cmakeVersion} | `
               Join-Path -ChildPath "bin" | `
               Join-Path -ChildPath $cmake

      if (!(Test-Path($cmake)))
      {
         Write-Host "${cmake} does not exist" -ForegroundColor Red
         exit
      }
   }
   elseif ($IsMacOS)
   {
      # Android SDK for OSX does not have cmake?
      $cmake = "cmake"
   }

   $androidNdkHome = $env:ANDROID_NDK_HOME
   if (!($androidNdkHome))
   {
      Write-Host "ANDROID_NDK_HOME is missing" -ForegroundColor Red
      exit
   }
   
   if (!(Test-Path($androidNdkHome)))
   {
      Write-Host "ANDROID_NDK_HOME: ${androidNdkHome} does not exist" -ForegroundColor Red
      exit
   }

   $toolchain = Join-Path $androidNdkHome "build" | `
               Join-Path -ChildPath "cmake" | `
               Join-Path -ChildPath "android.toolchain.cmake"
   if (!(Test-Path($toolchain)))
   {
      Write-Host "${toolchain} does not exist" -ForegroundColor Red
      exit
   }

   & "${cmake}" -G Ninja `
                -D CMAKE_BUILD_TYPE=${configuration} `
                -D CMAKE_INSTALL_PREFIX=${installDir} `
                -D CMAKE_TOOLCHAIN_FILE="${toolchain}" `
                -D BUILD_SHARED_LIBS="ON" `
                -D ANDROID_CPP_FEATURES="exceptions" `
                -D ANDROID_ALLOW_UNDEFINED_SYMBOLS="TRUE" `
                -D ANDROID_STL="c++_static" `
                -D ANDROID_NDK="${androidNdkHome}"  `
                -D ANDROID_NATIVE_API_LEVEL=$androidNativeApiLevel `
                -D ANDROID_ABI="${ANDROID_ABI}"  `
                $sourceDir
   & "${cmake}" --build . --config ${configuration} --target install
}
else
{
   $rootDir = (git rev-parse --show-superproject-working-tree --show-toplevel)
   $toolchainDir = Join-Path $rootDir "toolchains"
   if (!(Test-Path($toolchainDir)))
   {
      Write-Host "Error: ${toolchainDir} is missing" -ForegroundColor Red
      exit
   }
   
   $toolchain = Join-Path $toolchainDir "${architecture}-ios.cmake"
   cmake -D CMAKE_BUILD_TYPE=$configuration `
         -D CMAKE_INSTALL_PREFIX=${installDir} `
         -D BUILD_SHARED_LIBS="ON" `
         -D CMAKE_EXE_LINKER_FLAGS="-std=c++17 -stdlib=libc++" `
         -D CMAKE_IOS_INSTALL_COMBINED="NO" `
         -D CMAKE_OSX_SDK="${target}" `
         -D CMAKE_OSX_DEPLOYMENT_TARGET="${macosDeplolymentTarget}" `
         -D CMAKE_TOOLCHAIN_FILE="${toolchain}" `
         "${sourceDir}"
   cmake --build . --config ${configuration} --target install
}
Pop-Location