$curret = Get-Location

function Build()
{
   Param([string]$target)

   $sourceDir = Join-Path $curret sources | `
                Join-Path -ChildPath $target

   # $CMAKE_OSX_ARCHITECTURES="armv7;armv7s;arm64;i386;x86_64"
   # device
   Write-Host "Build iOS $target Device Library" -Foreground Green
   $buildDir = Join-Path $curret build | `
               Join-Path -ChildPath $target | `
               Join-Path -ChildPath ios_device_static
   $installDir = Join-Path $curret install | `
                 Join-Path -ChildPath $target | `
                 Join-Path -ChildPath ios_device_static
   $lib = Join-Path $installDir lib | `
          Join-Path -ChildPath lib"$target".a
   New-Item -ItemType Directory -Force $buildDir | Out-Null
   Push-Location $buildDir
   $CMAKE_OSX_ARCHITECTURES="arm64;arm64e"
   $CMAKE_IOS_INSTALL_COMBINED="NO"
   $CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH="NO"
   cmake -G Xcode `
         -D CMAKE_SYSTEM_NAME=iOS `
         -D CMAKE_OSX_ARCHITECTURES=$CMAKE_OSX_ARCHITECTURES `
         -D CMAKE_OSX_DEPLOYMENT_TARGET=9.3 `
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
               Join-Path -ChildPath ios_simulator_static
   $installDir = Join-Path $curret install | `
                 Join-Path -ChildPath $target | `
                 Join-Path -ChildPath ios_simulator_static
   $lib = Join-Path $installDir lib | `
          Join-Path -ChildPath lib"$target".a
   New-Item -ItemType Directory -Force $buildDir | Out-Null
   Push-Location $buildDir
   $CMAKE_OSX_ARCHITECTURES="x86_64"
   $CMAKE_IOS_INSTALL_COMBINED="NO"
   $CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH="NO"
   cmake -G Xcode `
         -D CMAKE_SYSTEM_NAME=iOS `
         -D CMAKE_OSX_SYSROOT="iphonesimulator" `
         -D CMAKE_OSX_ARCHITECTURES=$CMAKE_OSX_ARCHITECTURES `
         -D CMAKE_OSX_DEPLOYMENT_TARGET=9.3 `
         -D CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH=$CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH `
         -D CMAKE_IOS_INSTALL_COMBINED=$CMAKE_IOS_INSTALL_COMBINED `
         -D CMAKE_INSTALL_PREFIX=${installDir} `
         -D BUILD_FRAMEWORK=OFF `
         "${sourceDir}"
   cmake --build . --config Release --target install
   lipo -info $lib
   Pop-Location
}

Build NativeAdd

Write-Host "Build iOS Managed Library" -Foreground Green

$sourceDir = Join-Path $curret sources | Join-Path -ChildPath NativeSharp
Set-Location $sourceDir
dotnet build -c "Release_Xamarin.iOS"

Set-Location $curret