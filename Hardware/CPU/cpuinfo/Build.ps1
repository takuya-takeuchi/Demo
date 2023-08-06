#***************************************
#Arguments
#%1: Build Target (win/linux/osx/iphoneos/iphonesimulator/android)
#%2: Architecture (x86_64/armv7/arm64)
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
   "armv7",
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
$androidNativeApiLevel = 21
$androidNativeApiLevel2 = 28

$sourceDir = Join-Path $current $buildTarget
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $architecture
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $architecture

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

# restore
git submodule update --init --recursive .

Push-Location $buildDir

switch ($os)
{
    "win"
    {
        cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
              -D CMAKE_BUILD_TYPE=${configuration} `
              -D TARGET_ARCHITECTURES="${architecture}"  `
              $sourceDir
        cmake --build . --config ${configuration} --target install
    }
    "linux"
    {
        cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
              -D CMAKE_BUILD_TYPE=${configuration} `
              -D TARGET_ARCHITECTURES="${architecture}"  `
              $sourceDir
        cmake --build . --config ${configuration} --target install
    }
    "osx"
    {
        cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
              -D CMAKE_BUILD_TYPE=${configuration} `
              -D MACOSX_DEPLOYMENT_TARGET="${macosDeplolymentTarget}" `
              -D CMAKE_OSX_ARCHITECTURES="$architecture" `
              -D TARGET_ARCHITECTURES="${architecture}"  `
              -D BUILD_SHARED_LIBS="ON" `
              -D CPUINFO_BUILD_TOOLS="OFF" `
              -D CPUINFO_BUILD_UNIT_TESTS="OFF" `
              -D CPUINFO_BUILD_MOCK_TESTS="OFF" `
              -D CPUINFO_BUILD_BENCHMARKS="OFF" `
              $sourceDir
        cmake --build . --config ${configuration} --target install
    }
    "iphoneos"
    {
        $repoRootDir = (git rev-parse --show-toplevel)
        $toolchain = Join-Path $repoRootDir "toolchains" | `
                     Join-Path -ChildPath "${architecture}-ios.cmake"
        cmake -D CMAKE_BUILD_TYPE=${configuration} `
              -D CMAKE_INSTALL_PREFIX=${installDir} `
              -D BUILD_SHARED_LIBS="ON" `
              -D CMAKE_TOOLCHAIN_FILE="${toolchain}" `
              -D CMAKE_OSX_SDK="${os}" `
              -D CPUINFO_BUILD_TOOLS="OFF" `
              -D CPUINFO_BUILD_UNIT_TESTS="OFF" `
              -D CPUINFO_BUILD_MOCK_TESTS="OFF" `
              -D CPUINFO_BUILD_BENCHMARKS="OFF" `
              $sourceDir
        cmake --build . --config ${configuration} --target install
    }
    "iphonesimulator"
    {
        $repoRootDir = (git rev-parse --show-toplevel)
        $toolchain = Join-Path $repoRootDir "toolchains" | `
                     Join-Path -ChildPath "${architecture}-ios.cmake"
        cmake -D CMAKE_BUILD_TYPE=${configuration} `
              -D CMAKE_INSTALL_PREFIX=${installDir} `
              -D BUILD_SHARED_LIBS="ON" `
              -D CMAKE_TOOLCHAIN_FILE="${toolchain}" `
              -D CMAKE_OSX_SDK="${os}" `
              -D CPUINFO_BUILD_TOOLS="OFF" `
              -D CPUINFO_BUILD_UNIT_TESTS="OFF" `
              -D CPUINFO_BUILD_MOCK_TESTS="OFF" `
              -D CPUINFO_BUILD_BENCHMARKS="OFF" `
              $sourceDir
        cmake --build . --config ${configuration} --target install
    }
    "android"
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
        }
        elseif ($IsLinux)
        {
            $cmake = "cmake"
        }
        elseif ($IsMacOS)
        {
            $cmake = "cmake"
        }

        $cmake = Join-Path $androidHome "cmake" | `
                 Join-Path -ChildPath "3.22.1" | `
                 Join-Path -ChildPath "bin" | `
                 Join-Path -ChildPath $cmake
        if (!(Test-Path($cmake)))
        {
            Write-Host "${cmake} does not exist" -ForegroundColor Red
            exit
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

        switch ($architecture)
        {
            "armv7"  { $ANDROID_ABI = "armeabi-v7a" }
            "arm64"  { $ANDROID_ABI = "arm64-v8a" }
            "x86_64" { $ANDROID_ABI = "x86_64" }
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
                     -D CPUINFO_BUILD_TOOLS="OFF" `
                     -D CPUINFO_BUILD_UNIT_TESTS="OFF" `
                     -D CPUINFO_BUILD_MOCK_TESTS="OFF" `
                     -D CPUINFO_BUILD_BENCHMARKS="OFF" `
                     $sourceDir
        & "${cmake}" --build . --config ${configuration} --target install
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