$curret = Get-Location

function BuildAndroid()
{
      Param([string]$target)

      $androidNativeApiLevel = 21
      $androidNativeApiLevel2 = 28

      $sourceDir = Join-Path $curret sources | `
      Join-Path -ChildPath $target

      # device
      Write-Host "Build Android $target Library" -Foreground Green
      $buildDir = Join-Path $curret build | `
      Join-Path -ChildPath $target | `
      Join-Path -ChildPath android
      $installDir = Join-Path $curret install | `
      Join-Path -ChildPath $target | `
      Join-Path -ChildPath android
      $lib = Join-Path $installDir lib | `
      Join-Path -ChildPath lib"$target".a
      New-Item -ItemType Directory -Force $buildDir | Out-Null
      Push-Location $buildDir

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

      $androidNdkHome = $env:ANDROID_NDK_ROOT
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
                   -D CMAKE_BUILD_TYPE=Release `
                   -D CMAKE_INSTALL_PREFIX=${installDir} `
                   -D CMAKE_TOOLCHAIN_FILE="${toolchain}" `
                   -D BUILD_SHARED_LIBS="ON" `
                   -D ANDROID_CPP_FEATURES="exceptions" `
                   -D ANDROID_ALLOW_UNDEFINED_SYMBOLS="TRUE" `
                   -D ANDROID_STL="c++_static" `
                   -D ANDROID_NDK="${androidNdkHome}"  `
                   -D ANDROID_NATIVE_API_LEVEL=$androidNativeApiLevel `
                   -D ANDROID_ABI="arm64-v8a"  `
                   -D CPUINFO_BUILD_TOOLS="OFF" `
                   -D CPUINFO_BUILD_UNIT_TESTS="OFF" `
                   -D CPUINFO_BUILD_MOCK_TESTS="OFF" `
                   -D CPUINFO_BUILD_BENCHMARKS="OFF" `
                   "${sourceDir}"
      cmake --build . --config Release --target install
      Pop-Location
}

function BuildIOS()
{
      Param([string]$target)

      $sourceDir = Join-Path $curret sources | `
      Join-Path -ChildPath $target

      # $CMAKE_OSX_ARCHITECTURES="armv7;armv7s;arm64;i386;x86_64"
      # device
      Write-Host "Build iOS $target Device Library" -Foreground Green
      $buildDir = Join-Path $curret build | `
      Join-Path -ChildPath $target | `
      Join-Path -ChildPath ios_device
      $installDir = Join-Path $curret install | `
      Join-Path -ChildPath $target | `
      Join-Path -ChildPath ios_device
      $lib = Join-Path $installDir lib | `
      Join-Path -ChildPath lib"$target".a
      New-Item -ItemType Directory -Force $buildDir | Out-Null
      Push-Location $buildDir
      $CMAKE_OSX_ARCHITECTURES="arm64;arm64e"
      $CMAKE_IOS_INSTALL_COMBINED="NO"
      $CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH="NO"
      cmake -G Xcode `
            -D CMAKE_BUILD_TYPE=Release `
            -D CMAKE_SYSTEM_NAME=iOS `
            -D CMAKE_OSX_ARCHITECTURES=$CMAKE_OSX_ARCHITECTURES `
            -D CMAKE_OSX_DEPLOYMENT_TARGET=11.0 `
            -D CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH=$CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH `
            -D CMAKE_IOS_INSTALL_COMBINED=$CMAKE_IOS_INSTALL_COMBINED `
            -D CMAKE_INSTALL_PREFIX=${installDir} `
            -D BUILD_FRAMEWORK=OFF `
            "${sourceDir}"
      cmake --build . --config Release --target install
      lipo -info $lib
      Pop-Location

      # simulator
      Write-Host "Build iOS $target Simulator Library" -Foreground Green
      $buildDir = Join-Path $curret build | `
      Join-Path -ChildPath $target | `
      Join-Path -ChildPath ios_simulator
      $installDir = Join-Path $curret install | `
      Join-Path -ChildPath $target | `
      Join-Path -ChildPath ios_simulator
      $lib = Join-Path $installDir lib | `
      Join-Path -ChildPath lib"$target".a
      New-Item -ItemType Directory -Force $buildDir | Out-Null
      Push-Location $buildDir
      # for M1, M2 or later mac
      $CMAKE_OSX_ARCHITECTURES="arm64;arm64e"
      $CMAKE_IOS_INSTALL_COMBINED="NO"
      $CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH="NO"
      cmake -G Xcode `
            -D CMAKE_SYSTEM_NAME=iOS `
            -D CMAKE_OSX_SYSROOT="iphonesimulator" `
            -D CMAKE_OSX_ARCHITECTURES=$CMAKE_OSX_ARCHITECTURES `
            -D CMAKE_OSX_DEPLOYMENT_TARGET=11.0 `
            -D CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH=$CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH `
            -D CMAKE_IOS_INSTALL_COMBINED=$CMAKE_IOS_INSTALL_COMBINED `
            -D CMAKE_INSTALL_PREFIX=${installDir} `
            -D BUILD_FRAMEWORK=OFF `
            "${sourceDir}"
      cmake --build . --config Release --target install
      lipo -info $lib
      Pop-Location
}

BuildAndroid NativeAdd
BuildIOS NativeAdd

Write-Host "Build Managed Library" -Foreground Green

$sourceDir = Join-Path $curret sources | Join-Path -ChildPath NativeSharp
Set-Location $sourceDir
dotnet build -c "Release_iOS"
dotnet build -c "Release"

Set-Location $curret